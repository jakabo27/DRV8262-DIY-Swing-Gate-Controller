# Host-side tests

The two firmwares talk over a text protocol: the MSP430 prints `EVT …` lines and the ESP32 parses them. If one side renames a field and the other doesn't, both still build, and Home Assistant quietly shows a stale value. These tests run on a PC and catch that.

| Test | What it does |
|---|---|
| `test_beam.c` | Compiles the **real** `beam.c` against stubs and feeds it acceptance waveforms: a 200 ms hand wave, two breaks 1 s apart, a car body, a car plus trailer, a 10 s block, a 20 ms glitch, chatter. It runs twice, with the axle-pair classifier off (the default) and on |
| `test_esp_parsers.cpp` | Compiles the bridge's parsers **sliced verbatim out of** `esp32-wifi-bridge/src/main.cpp` by `gen_esp_test.py`, so the test can't drift from the shipping code. Checks field extraction, retain flags, the chatter rate limit and JSON payload shape |
| `test_wire_contract.py` | Reads both firmwares' sources and checks that every `key=` the bridge parses is one the MSP430 emits, every event type is declared in HA discovery, and no event body contains the `;` message terminator. It also guards two bugs found on the bench: idle motor current latching at the last value, and the ADC stall watchdog |

Run all three (Windows, needs Visual Studio Build Tools with the C++ workload, and Python 3):

```powershell
powershell -ExecutionPolicy Bypass -File tests\run_tests.ps1
```
