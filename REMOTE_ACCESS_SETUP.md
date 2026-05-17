# ANUBIX Remote Access Setup

This document is the **single source of truth** for running the ANUBIX agent on
the Jetson while driving it from a laptop on a **different network** through
the hosted OmniLink web UI at <https://www.omnilink-agents.com>.

It is grounded section-by-section in **OmniLink Remote Agent Access v1.0.0**
(the official integrator guide). Every choice here cites the corresponding
section.

---

## The constraint set we're solving for

- The Jetson and the operator laptop are on **different networks** (e.g.
  Jetson on a robotics lab subnet, laptop on home/campus Wi-Fi).
- The operator must be free to **switch laptops** without per-device setup.
- Only the **hosted** OmniLink UI is in play — no self-hosted UI, no
  Tailscale client on every laptop, no SSH tunnels typed by hand.

Per the documentation's decision table (§7), this corresponds to **"Remote
demo or one-off access"** / **"Always-on remote control"**. The only option
in that table that gives the operator zero per-laptop setup while still
satisfying the browser's HTTPS rule is **§5.1 Option E — HTTPS Tunnel**.

We use **Cloudflare Tunnel quick mode** specifically because the docs note it
is "free, no signup needed for quick mode" and produces a real, trusted
HTTPS URL. The OmniLink web UI can fetch it directly from the browser.

---

## Why this works (the rule, from §3 and §10)

> "The agent's URL, as the browser sees it, must be localhost / 127.0.0.1 /
> [::1] or an `https://` URL with a trusted certificate."

A Cloudflare quick tunnel URL like `https://random-words-1234.trycloudflare.com`
is HTTPS with a real Let's Encrypt certificate — the browser is happy, the
mixed-content rule (§3) does not apply, and the operator's laptop needs
**nothing** installed.

---

## What runs where

Per the mental model in §1 of the doc:

| Actor | Where | What it does |
|---|---|---|
| **Web UI** | Operator's browser, on the hosted page | Chat + sends tool calls |
| **Agent process** | Jetson (`anubix_master` node) | Runs the tool-callback HTTP server on port 5055 |
| **Cloudflare Tunnel** | Same Jetson, loopback | Publishes `http://localhost:5055` as a public HTTPS URL |
| **Hardware bridges** | Jetson + RPi (via CycloneDDS) | Arm, vision, spectrometer, navigation, Supabase |

ANUBIX is a **ToolRunner-style agent** (per §2.2), so the OmniLink chat
itself is handled by the OmniLink platform's backend — we only need to make
our **tool-callback server** reachable. That means **one tunnel, one port**:
5055. (The doc's reminder in §6.2 — "if you tunnel one, tunnel both" —
applies to agents that also expose an OmniLinkHTTPBridge; ANUBIX does not.)

---

## One-time Jetson setup

```bash
# 1. Clone the repos onto the Jetson (if not already)
git clone https://github.com/AbdelrahmanAtef01/ANUBIX_JETSON_WS.git anubix_ws

# 2. Install cloudflared (per docs §5.1)
cd anubix_ws
sudo ./scripts/install_cloudflared.sh

# 3. Build the ROS 2 workspace
colcon build
source install/setup.bash

# 4. Export your OmniLink key
export OMNI_KEY=olink_4ekYIgHACfZaGlq6WJOgu59U   # or your own key
```

That's it. **No SSH keys, no port forwarding rules on the lab router, no DNS,
no certificates to import on operator laptops.**

---

## Per-session startup (the normal flow)

On the Jetson:

```bash
source install/setup.bash
export OMNI_KEY=olink_...
./scripts/start_anubix_remote.sh
```

What you'll see on stdout:

```
[CLOUDFLARED] quick tunnel established: https://forty-rain-2024.trycloudflare.com
[CALLBACK] toolCallbackUrl: https://forty-rain-2024.trycloudflare.com/tool
[PROFILE] Updated ANUBIX profile (ID: ...)
[PROFILE] toolCallbackUrl = https://forty-rain-2024.trycloudflare.com/tool

======================================================================
  ANUBIX READY
======================================================================
```

On **any** laptop with internet access:

1. Open <https://www.omnilink-agents.com>
2. Sign in (or use a shared account)
3. Select **ANUBIX** from the agent dropdown — its profile is already
   pointing at the live tunnel URL because the master node just re-registered
   it on startup.
4. Send a task in the chat, e.g.
   `Check disease at (3, 5) with robot_id=abc and task_id=def`
5. The AI emits tool calls → the browser POSTs them to the tunnel URL →
   cloudflared forwards to the Jetson → the master node executes them via
   ROS 2 → results stream back.

---

## Why the URL changes each run (and why that's fine)

Quick tunnels (the no-signup variety, §5.1) rotate their public URL on every
`cloudflared` restart. The doc flags this:

> "Free tunnel URLs rotate on restart."

We handle this automatically: the master node always re-registers the live
URL into the **ANUBIX agent profile** (the `toolCallbackUrl` field) at
startup, before announcing itself ready. The operator never needs to know
or paste the URL.

If you want a stable URL — for example, a permanent installation — the
doc's prescription in §5.1 is:

> "For long-running setups, use named Cloudflare tunnels with a stable
> hostname (`cloudflared tunnel create` + a DNS record on your zone) so the
> URLs don't rotate on restart."

To do that, set `ANUBIX_TOOL_CALLBACK_URL` to the stable HTTPS URL of your
named tunnel and the master node will skip starting a quick tunnel.

---

## Security note (§6 of the doc)

The bridge ships with no auth and `CORS: *`. The doc is explicit:

> "Anywhere [other than loopback], the bridge must sit behind an auth layer.
> If you skip this on a remote setup, anyone who guesses or scrapes the URL
> can issue commands to the agent — and for a robot agent, that includes
> motion commands."

Quick tunnel URLs are essentially unguessable (long random subdomains), so
they are effectively a bearer secret. Treat the URL as a credential:

- Don't paste it into Slack or commit it to git.
- Take the tunnel down (`Ctrl+C` the master node) when you're done.
- For a graduation-demo timeline, this is acceptable; for a production
  fleet, follow the doc's §6.1 ladder — Caddy basic auth in front of the
  tunnel, or step up to Option F (Tailscale + ACLs).

A **hardware e-stop** is still the law (§6.4) — the bridge is the control
plane, not the safety plane.

---

## Why only ONE tunnel, not two

The Remote Agent Access doc §2 distinguishes two HTTP services:

| Service | Port | Class | Used by |
|---|---|---|---|
| **Automation bridge** (§2.1) | 5000 | `OmniLinkHTTPBridge` | The Web UI's **Connection panel** (in self-hosted setups, or when running a packaged agent like Axis/Haven) |
| **Tool-callback server** (§2.2) | per-agent (Axis: 51516, ToolRunner: random) | Custom HTTP handler | The Web UI **when the AI emits a structured tool call** |

ANUBIX follows the **ToolRunner pattern** (per Python Library §2 and Agents PDF p.85 Robot Demo architecture): no `OmniLinkHTTPBridge` is instantiated, the master node only opens a single HTTP server on port 5055. So:

- §6.2's reminder *"if you tunnel one, tunnel both"* applies to standard agents that **run both services**. ANUBIX only runs the tool-callback server.
- There is no port-5000 service on the Jetson to tunnel — tunneling it would forward to a closed port.

The browser flow for our setup is exactly the one drawn in Agents PDF p.83:

```
Operator browser ──(chat)──► OmniLink Platform API
       │                            │
       │  ◄──── structured toolCall ◄┘  (no bridge involved)
       │
       └──── POST /tool ────► Cloudflare Tunnel ────► Jetson:5055
```

---

## Debugging "Tool callback server isn't reachable (Failed to fetch)"

If you see this error in the OmniLink UI, work through this list **in order**:

**1. From the operator laptop, hit the tunnel URL directly with curl.**

```bash
# Should return 200 with a small JSON banner now (we added GET / and GET /tool):
curl -i https://<your-tunnel>.trycloudflare.com/

# CORS preflight test (must return 204 with Access-Control-Allow-Origin: *):
curl -i -X OPTIONS \
     -H "Origin: https://www.omnilink-agents.com" \
     -H "Access-Control-Request-Method: POST" \
     -H "Access-Control-Request-Headers: content-type" \
     https://<your-tunnel>.trycloudflare.com/tool

# Actual POST test (must return 200 with a JSON body):
curl -i -X POST \
     -H "Content-Type: application/json" \
     -H "Origin: https://www.omnilink-agents.com" \
     -d '{"tool":"supervisor_force_stop"}' \
     https://<your-tunnel>.trycloudflare.com/tool
```

If **any** of these fails, the issue is between curl and the Jetson — not the browser. Common causes:
- Cloudflare quick tunnel hasn't propagated yet (wait 10 s after the master node prints `[CLOUDFLARED] quick tunnel established`).
- The master node's HTTP server crashed (check the master node terminal — the `[HTTP]` lines log every hit since this commit).
- The Jetson's HTTP server bound to `127.0.0.1` instead of `0.0.0.0` — but our default is `0.0.0.0`, so this is rare.

**2. If curl works but the UI still fails, check the browser DevTools (F12 → Network).**

Find the failed request to `/tool`. Look at:
- **Status column**: empty/red means CORS or CSP blocked it before sending. A numeric status (4xx/5xx) means it reached the server.
- **Console tab**: a CSP error reads `Refused to connect to '...' because it violates the following Content Security Policy directive: connect-src ...`. If you see this, the OmniLink platform's CSP is restricting the connection — and per Agents PDF p.106 ("Other origins — Wildcarded — allowed with standard restrictions") that shouldn't happen, but it's still worth verifying. Send a screenshot if you see it.
- **Preflight (OPTIONS)** vs **actual POST**: if preflight fails, the POST never happens. The fix in this commit echoes the requested headers back, which handles arbitrary OmniLink frontend headers.

**3. Read the master node's `[HTTP]` lines.**

Since this commit, every HTTP hit on the tool-callback server is logged at INFO level. If you click *Send* in the OmniLink UI and **nothing** shows up in the master node terminal, the request never reached us — that's either cloudflared (tunnel down) or browser/CSP (request blocked before sending). If you see preflight lines but no POST, it's a CORS-rejection on the preflight response.

**4. If all else fails: SSH-tunnel fallback (Option A from doc §4.1).**

This bypasses every "remote-network" complication. From the operator laptop:

```bash
ssh -L 5055:127.0.0.1:5055 user@<jetson-ip>
```

Then on the Jetson, start the master node with `--tunnel none`:

```bash
ros2 run anubix_master master_node --tunnel none
```

The toolCallbackUrl becomes `http://localhost:5055/tool` — guaranteed-reachable from the operator's browser because the SSH tunnel maps it. This costs you the "no laptop setup" benefit but eliminates every other variable.

---

## Troubleshooting (cross-reference to §9 of the doc)

| Symptom | What to check |
|---|---|
| Chat works but **tools never run / hang forever** | Open the ANUBIX profile in the OmniLink dashboard. Does `toolCallbackUrl` match what the master node printed on startup? It should — if not, the master node failed to update the profile (check `[PROFILE]` log lines). Paste the URL into your browser's address bar: you should get a 404 on `GET /tool` (only POST is accepted), **not** a connection error. |
| `mixed-content / blocked: insecure content` in DevTools | Profile is pointing at `http://...` instead of `https://...`. The tunnel didn't start — check the master node logs for `[CLOUDFLARED]` lines. |
| `cloudflared exited` on master node startup | Run `cloudflared --version` on the Jetson. If missing, run `scripts/install_cloudflared.sh`. If installed but exiting, check `/tmp/cloudflared.log` — usually a transient Cloudflare edge issue, restart fixes it. |
| Robot stops responding mid-session | Free tunnels can drop under load (§9 troubleshooting). The master node now logs `[TUNNEL] cloudflared process exited` if the daemon dies — restart the master node and the profile is re-registered with a fresh URL. |
| Old tunnel URL stuck in the profile | The master node only re-registers on startup. If you restart `cloudflared` manually, also restart the master node so the profile is refreshed. |

---

## Why we didn't pick the other options

(All numbered against the doc's §4 and §5.)

| Option | Why it's not the answer here |
|---|---|
| **A — SSH / socat port forward** | Requires `ssh -L 5055:...` on every operator laptop. User explicitly wants to switch laptops with no setup. |
| **B — Self-host the UI** | Same problem: every operator laptop needs the UI repo and `npm run dev`. |
| **C — Caddy + internal CA** | Requires importing Caddy's root CA into every operator laptop's OS trust store. Switching laptops = re-doing this. |
| **D — mDNS + Caddy** | Requires the laptop and Jetson to be on the **same broadcast domain**. They aren't. |
| **E — HTTPS tunnel (Cloudflare Tunnel)** | ✅ **Chosen.** Zero operator-laptop setup, real HTTPS cert, works from anywhere with internet, free. |
| **F — Tailscale** | Requires the Tailscale client + auth on every operator laptop. Doc calls this "best for always-on remote control," but it's incompatible with "switch laptops freely." |
| **G — Public DNS + Let's Encrypt** | Requires owning a domain, configuring DNS, and a permanent IP. Overkill for this project. |

---

## Files added/changed for this setup

- `src/anubix_master/anubix_master/ros_master_node.py` — added `--tunnel`
  flag (default `cloudflared`), `_start_cloudflared`, `_resolve_callback_url`,
  and tunnel lifecycle management in `main()`.
- `agent_config/configure_anubix_agent.py` — aligned env-var name with
  master node (`ANUBIX_TOOL_CALLBACK_URL`), updated next-steps blurb.
- `scripts/install_cloudflared.sh` — one-time Jetson install of
  cloudflared.
- `scripts/start_anubix_remote.sh` — convenience launcher.
- `REMOTE_ACCESS_SETUP.md` — this file.

The Raspberry Pi workspace (`anubix_rpi_ws`) is unchanged — it never speaks
to OmniLink directly. All OmniLink ↔ ROS 2 traffic is mediated by the master
node on the Jetson.
