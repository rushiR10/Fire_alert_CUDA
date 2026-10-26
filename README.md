# Fire Alert Server: CDA-GB graph burning on CUDA (simulation)

A small simulation of a hospital fire-alert server. A fire sensor reports a room. The server then decides,
round by round, which phone to alert directly, and phones relay the alert to nearby phones. The choice
uses the P-9 algorithm (Criticality- and Deadline-Aware Graph Burning). The candidate scoring runs on
the GPU with CUDA.

## How the idea maps to graph burning

| Graph burning (P-9) | In this project |
|---|---|
| node | a person's phone (doctor, nurse, patient, admin, visitor, coordinator) |
| temporal edge in round r | two phones close enough for Bluetooth / Wi-Fi Direct in that round |
| burning source (one per round) | the **server pushes** the alert directly to one phone |
| fire spreading to neighbours | phones that have the alert **relay** it to phones they meet |
| criticality c(v) | role (doctor 5, nurse 4, patient 3, admin 2, visitor 1) **+2 if in the fire room, +1 if next to it** (max 5) |
| deadline d(v) | 7 − c(v) rounds (1 round = 30 s) |
| source-selection score | Gain × (1 − 0.3 · Overlap), exactly Steps 4–7 of the proposal |

The server can push to only **one phone per round** because during a fire the building's network is
congested or damaged. Phone-to-phone relay does the rest. That is the graph-burning model.

## Where CUDA is used

At the start of each round the server must score every phone that has not been alerted yet: "if I push
to this phone now, who gets the alert before their deadline?" That is one temporal-reachability
simulation per candidate, O(N·E) per round.

`scoreKernel` in [src/firealert.cu](src/firealert.cu) runs **one thread block per candidate phone**. The
block keeps every phone's arrival round in shared memory. Its threads sweep that round's contacts in
parallel, then add up gain, reach and overlap. The host picks the best score. A CPU version of the same
scoring runs alongside and the screen shows `identical ✓`. Gain is summed as a scaled integer, so the GPU
and CPU results are bit-identical.

When the program is built without `nvcc`, or no NVIDIA GPU is found, it uses the CPU version. The
algorithm and results are the same. On 40 phones both paths take under a millisecond. Try
`--people 2000`: the CPU then needs about 0.3 s per plan, and that is where the GPU helps.

## Can it really alert phones?

Yes. In this simulation each phone is a small terminal program that connects to the server over TCP and
waits. A real phone app does the same thing. For a real deployment you would swap the transport:

* **Server → phone (the "push"):** Firebase Cloud Messaging (Android) or APNs (iOS) push notifications,
  an SMS gateway, the hospital's Wi-Fi VoIP phones or pagers, or a WebSocket kept open by the app.
* **Phone → phone (the "relay"):** Bluetooth LE or Wi-Fi Direct between nearby phones. Android
  "Nearby Connections" and Apple "Multipeer Connectivity" provide this.
* **Contacts the server predicts:** indoor positioning (Wi-Fi access points or BLE beacons) tells the
  server who is near whom. This simulation generates it from a random walk between rooms.

## Build

**macOS (no CUDA, CPU planner):**
```bash
./build.sh
```

**Windows PC with an NVIDIA GPU** (CUDA 12.x + Visual Studio 2022 Build Tools):
```bat
build.bat
```

**Linux GPU server:**
```bash
./build.sh
```
`build.sh` uses `nvcc` automatically when it is on the PATH.

## Run 1: everything in one terminal

```bash
./bin/firealert demo --fire ICU
```

The hospital map is redrawn every round:

```
 Fire in ICU  |  round 2/10  t+01:00  |  40 people, 27 critical  |  server push budget K=4
 Planner: CUDA GPU NVIDIA GeForce GTX 1050
 ┌─────────────────────┬─────────────────────┬─────────────────────┬─────────────────────┐
 │ FIRE ICU            │ER  near fire        │OT                   │Radiology            │
 │ P05 P09 P12 P14     │ N06 N08 P01 P02 P07 │ D02 N03 N07         │ N05                 │
 ...
 ▸ Planner: GPU scored 36 candidate phones: kernel 0.031 ms | CPU check 0.042 ms, identical ✓
 ▸ SERVER ══push══▶ D05 (Doctor, Ward-B, criticality 5, deadline 2)
 ▸ Relay:  D05→C01  D05→D01  N07→D02  N07→N03  N06→N04 ...
 ▸ Progress: critical on time 13/27 (48%)  |  informed 14/40 (35%)  |  critical missed 7
```

Colours: **red background** = server pushed directly, **green** = got the alert in time, **yellow** =
got it late, **magenta** = critical and still waiting, **red** = critical and missed, **grey** = not
critical and not reached yet. An underline marks phones reached in this round.

At the end the program compares CDA-GB with the Degree, Criticality-only and Random baselines on the same
fire:

```
   method              critical on time     weighted timely  informed       avg arrival
   CDA-GB (this run)   18/27 (67%)          0.711            40/40          round 2.98
   Degree              16/27 (59%)          0.638            40/40          round 3.02
   Criticality-only    11/27 (41%)          0.490            40/40          round 3.40
   Random              14/27 (52%)          0.570            40/40          round 3.40
```

## Run 2: server, phones and sensor in separate terminals

**Terminal 1: the alert server.** It prints the list of phones and the room each person is in.
```bash
./bin/firealert server
```

**Terminal 2: your phones.** Pick names from the server's list.
```bash
./bin/firealert phone --id D01,P09,V04
```

**Terminal 3: the fire sensor.**
```bash
./bin/firealert sensor --room ICU
```

You can also type `fire ICU` straight into the server terminal.

The server animates the rounds. Each phone pops up an alert (with a terminal beep) when the alert reaches
it:

```
 ╔════════════════════════════════════════════════════════════════╗
 ║ FIRE ALERT  -  fire in ICU                                     ║
 ║ To: D01, now in Ward-B  (criticality 5)                        ║
 ║ Received in round 2 (t+01:00) relayed by phone D05             ║
 ║ Your deadline: round 2  ->  ON TIME                            ║
 ║ Action: Move critical patients out of danger and coordinate    ║
 ║ with the nurses.                                               ║
 ╚════════════════════════════════════════════════════════════════╝
```

The phones can run on another computer: `--host <server-ip> --port 9000`.

## Run 3: browser replay for the presentation

Double-click `show_replay.bat` (or `show_replay.bat ER` for another room). It runs one fire event on the GPU,
saves every round to `viewer/replay_data.js` and opens `viewer/replay.html` in your browser. No internet is
needed.

The page shows the hospital map round by round. A red arrow goes from the server to the phone it pushes to,
and green arrows show phone-to-phone relays. The side panel shows the CDA-GB score, the next-best phones,
the CUDA kernel time and the CPU check. At the bottom, CDA-GB is compared with the Degree, Random and
Criticality-only baselines.

Controls: **Space** play / pause, **← →** step one round, the slider jumps to any round. Hover a phone to see
its role, criticality and deadline. Hover a relay to highlight the two phones.

Any run can save a replay with `--export`:
```bash
bin/firealert demo --fire Lab --people 60 --export viewer/replay_data.js
bin/firealert server --export viewer/replay_data.js     # rewritten after every fire event
```
Reload the page after each run.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--fire` / `--room` | ICU | room on fire: ICU ER OT Radiology Ward-A NurseStn Ward-B Lab Pharmacy Lobby Admin Canteen |
| `--people` | 40 | number of people / phones (map shows names up to 60, counts above) |
| `--K` | 4 | number of direct server pushes |
| `--H` | 10 | rounds simulated |
| `--w` | 0.3 | overlap weight |
| `--fail` | 0 | fraction of contacts that silently fail (the server does not know) |
| `--seed` | 7 | same seed = same hospital on every machine |
| `--delay` | 1500 | ms between rounds (`0` = instant) |
| `--no-clear` | | print rounds one after another instead of redrawing |
| `--cpu` | | force the CPU planner on a GPU machine |
| `--no-verify` | | skip the CPU check of the GPU scores |
| `--no-color` | | plain text |
| `--export` | | save the event for the browser replay, e.g. `viewer/replay_data.js` |

## Message protocol (one line per message, TCP)

```
phone  → server   HELLO D01
server → phone    WELCOME|0|D01|Doctor|Ward-B
sensor → server   FIRE ICU
server → sensor   OK|fire in ICU reported to the alert server
server → phone    ALERT|round|DIRECT or RELAY|from|fireRoom|deadline|onTime|criticality|room|action|seconds
server → phone    END|ON_TIME or LATE or NOT_REACHED|fireRoom|arrivalRound|deadline
```

## Simplifications

* The hospital, movements and contacts are synthetic: 12 rooms in a 3 × 4 grid, people take random walks
  between neighbouring rooms, and about 2 phone contacts per person per round.
* The server is assumed to know the upcoming contacts, as in the proposal's temporal graph. `--fail`
  makes some of them fail without the server knowing.
* Relay through phones is simulated by the server, which sends each phone its message when the
  simulation says it arrives. Real phones would relay over Bluetooth or Wi-Fi Direct.
* The fire does not spread during the event, and the greedy choice is not guaranteed to be optimal.
#   F i r e _ a l e r t _ C U D A  
 