# RespiGuard

A chest-worn asthma monitor built around an ESP32-S3. It learns the wearer's own
baseline for oxygen saturation, breathing rate and heart rate, then flags an
episode when those drift away from that baseline while a wheeze classifier is
also firing. Movement suppresses the alert, so exertion is not mistaken for an
attack.

Hand-soldered from breakout modules. Total parts cost around ₹1,700.

## Where things stand

| | |
|---|---|
| Parts | Ordered 27 Aug 2026, awaiting delivery |
| Simulation | Working — reaches ALERT, exertion veto demonstrated |
| Self-test firmware | Written, compiles, untested on hardware |
| Real firmware | Written, compiles, untested on hardware |
| Recorder firmware | Written, compiles, untested on hardware |
| Wheeze model | Trained on ICBHI. AUC 0.747 |
| Enclosure | Modelled, STLs rendered, not printed |

Nothing in this project has run on real hardware yet. Everything compiles and
the model's export is verified against the trained network, but that is the
extent of it.

## What is where

| Folder | What it holds |
|---|---|
| `RespiGuard_selftest/` | Probes every sensor over raw I²C, prints a pass/fail table. Run this first. |
| `RespiGuard_firmware/` | The real firmware — detection logic, sensor drivers, the classifier. |
| `RespiGuard_recorder/` | Streams mic audio over USB, for collecting fine-tuning data. |
| `simulation/` | The Wokwi sketch and circuit the detection logic was proven in. |
| `enclosure/` | Parametric case in OpenSCAD, plus rendered STLs. |
| `docs/BRINGUP_GUIDE.html` | The assembly and test procedure. Open in a browser at the bench. |

The machine-learning work and the build tools sit on `D:`, deliberately outside
this OneDrive folder — the dataset is 1.8 GB and should not be syncing.

| Path | What it holds |
|---|---|
| `D:\respiguard-ml\` | Training pipeline, virtual environment, runs, results |
| `D:\datasets\icbhi\` | The ICBHI 2017 respiratory sound database |
| `D:\tools\arduino-cli\` | Compiles sketches without opening the IDE |
| `D:\tools\openscad\` | Renders the enclosure STLs |

## The order things happen in

**Now, before parts arrive.** Nothing left. Firmware, model, enclosure and the
bring-up procedure are all written.

**When parts arrive.** Follow `docs/BRINGUP_GUIDE.html`. Power first and alone,
then one sensor at a time with the self-test after each, then the real firmware.

**After that.** Record 50–100 normal breaths on the finished device with
`record.py`, fine-tune the classifier on them, re-export and re-verify. This is
the step that closes the gap between ICBHI's stethoscope recordings and a
microphone under clothing, and it is not optional.

## Rebuilding the model from scratch

    cd D:\respiguard-ml
    .venv\Scripts\python.exe prepare_icbhi.py --src D:\datasets\icbhi\extracted
    .venv\Scripts\python.exe train_wheeze.py --epochs 200 --lr 5e-4
    .venv\Scripts\python.exe export_weights.py
    .venv\Scripts\python.exe verify_export.py

`verify_export.py` is not a formality. It is what proves the header the firmware
compiles in computes the same thing as the network whose score you are quoting.
Run it after every export.

## On the numbers

The classifier scores AUC 0.747 on patients it never trained on. Sensitivity is
0.885 and specificity 0.391, at an operating point chosen to weight a missed
breath twice as heavily as a false alarm.

Those are ICBHI numbers — chest-contact stethoscope audio. The microphone on
this device sits under clothing. They are a ceiling to work down from, not a
prediction of what the device will do.

Similar products exist and are further along: Strados Labs' RESP biosensor and
Health Care Originals' ADAMM both do continuous wheeze detection, both with
clinical validation this project does not have. What is different here is the
price and that it is buildable by hand.

Validation against real patients runs through a clinician with written consent.
Until that happens it is future work, and should be described that way.
