# Hardware recordings

Normal breathing captured through the device's own INMP441 microphone. These
close the gap between the ICBHI stethoscope audio the classifier was trained on
and the sound the finished device actually hears through clothing.

Put files here with `record.py` from the machine-learning side:

    .venv\Scripts\python.exe record.py --label rest

It writes 30-second WAVs at 4 kHz mono. Copy them into the matching folder
below and push.

| Folder   | What to record            | How many |
|----------|---------------------------|----------|
| `rest/`  | sitting still             | 20-30    |
| `walk/`  | walking                   | 15-20    |
| `stairs/`| climbing stairs           | 15-20    |
| `sleep/` | lying down                | 10-15    |

`stairs/` matters most. Climbing stairs raises breathing rate and heart rate
and drops SpO2 slightly, which looks a great deal like an episode. Without
those recordings the device alarms on every flight of stairs.

Every one of these is a **negative** example — breathing that is not an asthma
attack. Nobody needs to fake a wheeze. The wheezing examples come from ICBHI.

More people is better than more recordings from one person, since everyone
breathes differently.
