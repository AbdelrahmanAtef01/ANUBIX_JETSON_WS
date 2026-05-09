#!/usr/bin/env python3
"""
ANUBIX Supabase Uploader — data/storage client
================================================
Extracted and cleaned from the original uploadToSupabase.py.
Handles all Supabase table inserts and storage uploads.
Import this module from the ROS node; do not run it directly.
"""

import os
import logging
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional

log = logging.getLogger('anubix_supabase')

try:
    from supabase import create_client, Client
    _SUPABASE_AVAILABLE = True
except ImportError:
    _SUPABASE_AVAILABLE = False
    log.warning(
        '[SUPABASE] supabase-py not installed — '
        'run: pip3 install supabase')


# ── Data models ───────────────────────────────────────────────────────────────

@dataclass
class ReadingModel:
    """Maps 1-to-1 to the Supabase 'readings' table columns."""
    robot_id: str
    task_id: str
    plant_location: str
    disease_detected: bool
    disease_name: str
    water_stress_level: float = 0.0
    harvest_details: Optional[str] = None
    photo_1_url: Optional[str] = None
    photo_2_url: Optional[str] = None
    recorded_at: Optional[str] = None

    def __post_init__(self):
        if self.recorded_at is None:
            self.recorded_at = datetime.now().isoformat()

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_spectral_result(
        cls,
        robot_id: str,
        task_id: str,
        plant_location: str,
        task_type: str,
        classification: str,
        value: float,
        photo_1_url: Optional[str] = None,
        photo_2_url: Optional[str] = None,
    ) -> 'ReadingModel':
        """
        Build a ReadingModel from a SpectralAnalyzer AnalysisResult.

        task_type mapping:
          'disease'       → disease_detected / disease_name
          'water_stress'  → water_stress_level
          'harvest_status'→ harvest_details
        """
        disease_detected    = False
        disease_name        = 'none'
        water_stress_level  = 0.0
        harvest_details     = None

        if task_type == 'disease':
            disease_detected = classification in ('early_stage', 'infected')
            # Default disease to TMV (Tobacco Mosaic Virus) when positive.
            # Update disease_name mapping as more pathogens are added.
            disease_name = 'TMV' if disease_detected else 'none'

        elif task_type == 'water_stress':
            water_stress_level = round(float(value), 4)

        elif task_type == 'harvest_status':
            harvest_details = classification

        return cls(
            robot_id=robot_id,
            task_id=task_id,
            plant_location=plant_location,
            disease_detected=disease_detected,
            disease_name=disease_name,
            water_stress_level=water_stress_level,
            harvest_details=harvest_details,
            photo_1_url=photo_1_url,
            photo_2_url=photo_2_url,
        )


# ── Supabase client wrapper ───────────────────────────────────────────────────

class SupabaseUploader:
    """
    Thin wrapper around the Supabase Python client.
    All methods log extensively — if something fails the log will tell you exactly why.
    """

    def __init__(self, url: str, key: str):
        if not _SUPABASE_AVAILABLE:
            raise RuntimeError(
                'supabase-py is not installed. '
                'Run: pip3 install supabase')

        if not url or not key:
            raise ValueError(
                f'Supabase URL and key must both be non-empty. '
                f'Got url={url!r}  key={"<set>" if key else "<empty>"}')

        self._client: Client = create_client(url, key)
        self._url = url
        log.info(f'[SUPABASE] Client initialized — project: {url}')

    # ── Table operations ──────────────────────────────────────────────────────

    def upload_reading(self, reading: ReadingModel) -> bool:
        """
        Insert a ReadingModel row into the 'readings' table.
        Returns True on success, False on any error.
        """
        try:
            data = reading.to_dict()

            log.info(
                f'[SUPABASE] Inserting into readings: '
                f'robot={reading.robot_id}  '
                f'task={reading.task_id}  '
                f'location={reading.plant_location!r}  '
                f'disease_detected={reading.disease_detected}  '
                f'disease_name={reading.disease_name!r}  '
                f'water_stress={reading.water_stress_level:.4f}  '
                f'harvest={reading.harvest_details!r}  '
                f'photo_1={reading.photo_1_url!r}  '
                f'photo_2={reading.photo_2_url!r}  '
                f'recorded_at={reading.recorded_at!r}')

            response = self._client.table('readings').insert(data).execute()

            log.info(
                f'[SUPABASE] INSERT SUCCESS  '
                f'response_data={getattr(response, "data", response)}')
            return True

        except Exception as e:
            log.error(
                f'[SUPABASE] INSERT FAILED: {type(e).__name__}: {e}  '
                f'row_data={reading.to_dict()}')
            return False

    # ── Storage operations ────────────────────────────────────────────────────

    def upload_image(
        self,
        file_path: str,
        bucket: str = 'plant-images',
    ) -> Optional[str]:
        """
        Upload a local image to Supabase Storage.
        Returns the public URL string, or None on failure.
        Deletes the local file after a successful upload.
        """
        if not os.path.exists(file_path):
            log.error(f'[SUPABASE] Image not found: {file_path!r}')
            return None

        try:
            ext = file_path.rsplit('.', 1)[-1].lower()
            unique_name = (
                f'scan_{datetime.now().strftime("%Y%m%d_%H%M%S")}.{ext}')

            log.info(
                f'[SUPABASE] Uploading image: {file_path!r} → '
                f'bucket={bucket!r}  name={unique_name!r}')

            with open(file_path, 'rb') as f:
                self._client.storage.from_(bucket).upload(
                    path=unique_name,
                    file=f,
                    file_options={'content-type': f'image/{ext}'},
                )

            public_url = (
                self._client.storage.from_(bucket).get_public_url(unique_name))

            log.info(f'[SUPABASE] Image uploaded  url={public_url!r}')

            try:
                os.remove(file_path)
                log.info(f'[SUPABASE] Local file deleted: {file_path!r}')
            except OSError as del_err:
                log.warning(
                    f'[SUPABASE] Could not delete local file {file_path!r}: {del_err}')

            return public_url

        except Exception as e:
            log.error(
                f'[SUPABASE] Image upload FAILED {file_path!r}: '
                f'{type(e).__name__}: {e}')
            return None
