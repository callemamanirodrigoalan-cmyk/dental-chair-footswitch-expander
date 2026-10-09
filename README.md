# Dental Chair Foot-Switch Expander — 2 → 12 Functions (LGT8F328P)

Firmware for a **retrofit controller** that expands a dental chair's foot switch from **2 to 12
functions** — without modifying the chair's original control electronics. The controller drives
the chair's existing panel switches electronically through **optocoupler-isolated outputs**, and
accepts commands from **two independent sources at once**: 12 physical switches and a UART/Bluetooth
link.

> The original foot pedal exposed only 2 functions. The clinic needed 12 — the original two plus
> 8 new buttons and direct control of the chair's seat and backrest motors (up / down). This
> controller adds them by **emulating the panel switches** through isolated outputs, so nothing in
> the chair's original board is altered or put at risk.

---

## How it works

Instead of modifying the chair's control board, the system taps its front-panel switches in
parallel through **PC817 optocouplers** (galvanic isolation — the retrofit and the chair share no
electrical connection). A microcontroller decides which "switches" to press, driving the
optocouplers through two **PCF8574 I²C port expanders**.

```
  ┌─────────────────────┐          ┌───────────────────────┐
  │  12 PHYSICAL SW      │          │  UART (HC-05 / FTDI)  │
  │  debounced inputs    │          │  9600 8N1             │
  └──────────┬──────────┘          └──────────┬────────────┘
             │                                │  ring-buffer ISR + parser
             ▼                                ▼
       phys_state[12]                  serial_active[12]   ← keep-alive timers
             └───────────────┬────────────────┘
                             ▼
             state_final[i] = phys_state[i] OR serial_active[i]
                             ▼
             pack 12 bits → invert (sink mode) → send on change only
                             ▼
             I²C → 2× PCF8574 (0x20 / 0x21) → PC817 optocouplers
                             ▼
             chair's original panel switches (isolated)
```

**Two boards:** Board 1 is the controller (switches + UART + MCU); Board 2 carries the two
PCF8574 expanders and the PC817 optocoupler array that interfaces the chair's panel.

---

## Engineering highlights

| Area | What it does | Why it matters |
|---|---|---|
| **Dual command source** | Physical switches and UART are merged with a per-channel **OR** | Either source can activate a function; **neither can cancel what the other requests** |
| **Galvanic isolation** | PC817 optocouplers in sink mode drive the chair's panel switches | The retrofit is electrically isolated from the chair — no risk to the original electronics |
| **Keep-alive "deadman switch"** | Each UART channel auto-releases 300 ms after the last `SWn TRUE` | If Bluetooth drops mid-command, the function releases safely on its own |
| **Differential I²C** | A port expander is written **only when its state changes** | The bus is idle at rest — no needless traffic or EMI, and room for other masters |
| **Non-blocking loop** | 5 fixed blocks per iteration, no `delay()` in the hot path | Measured worst-case iteration ~1.2 ms (with I²C), far under the watchdog window |
| **Counter-based debounce** | 3 consecutive consistent samples (~15 ms window) | Rejects mechanical chatter *and* periodic noise bursts (e.g. chair-motor EMI) |
| **Robust UART parser** | Ring-buffer ISR → line framing with overflow & partial-line timeout → tokenized dispatch | Survives a dropped/garbled Bluetooth link without hanging |
| **Self-healing I²C** | 3 retries with backoff; on failure `last_sent` is not updated, so it retries next loop | A transient bus fault recovers automatically |
| **Fail-safe everywhere** | Safe state (all off) at boot before the watchdog arms; 2 s WDT fed every iteration | The system fails to a safe state rather than hanging with outputs active |

---

## UART command set

Case-insensitive, terminated by `\n` or `\r`:

| Command | Argument | Action |
|---|---|---|
| `SWn` | `TRUE` / `1` / `ON` / *(none)* | Refresh keep-alive for channel *n* (1–12) |
| `SWn` | `FALSE` / `0` / `OFF` | Release that channel immediately |
| `STATUS` | — | Dump per-source and combined state |
| `SCAN` | — | Scan the I²C bus (0x08–0x77) |
| `ALLOFF` | — | Cancel all 12 keep-alives |
| `HELP` | — | List commands |

Recommended sender protocol: `SWn TRUE` every 100 ms (tolerates 2 lost messages before release;
max release latency ≤300 ms after the key is let go).

---

## Hardware

- **MCU:** LGT8F328P Pro Mini @ 32 MHz / 5 V
- **I/O expansion:** 2× PCF8574 (I²C, 0x20 and 0x21)
- **Isolation / output:** PC817 optocouplers (sink mode) driving the chair's panel switches
- **Comms:** HC-05 Bluetooth **or** FTDI, 9600 8N1 (one at a time on the UART)
- **Inputs:** 12 physical switches with debounce
- **Custom PCBs:** designed in EasyEDA / KiCad (see `/hardware`)

### Resource usage
- Flash: ~8 kB / 32 kB
- RAM: ~250 B / 2 kB
- Latency (physical switch → output): ~16 ms · (UART → output): ~2 ms

---

## Repository layout

```
/src        Firmware (Board1_v2_uart_keepalive.ino)
/hardware   PCB design files and board images
/docs       Architecture notes
```

---

## Skills demonstrated

Embedded C/C++ · LGT8F328P / AVR · I²C (PCF8574 port expanders) · optocoupler interfacing &
galvanic isolation · UART protocol design & robust parsing · cooperative multitasking (non-blocking
loop) · debouncing · watchdog & fail-safe design · keep-alive / deadman-switch logic · custom PCB
design (EasyEDA/KiCad) · medical-equipment retrofitting.

---

*Retrofit controller developed for a real dental chair. Board photos and a short demo video will
be added.*
