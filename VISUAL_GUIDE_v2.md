# 🚀 Quick Visual Guide - SPIN DOCTOR v2.0.0

## Application Overview

```
┌─────────────────────────────────────────────────────────────────┐
│ ⚡ SPIN DOCTOR - BLDC Motor Control Simulator (v2.0.0)          │
├─────────────────────────────────────────────────────────────────┤
│ File  Tools  Help                                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ⚡ SPIN DOCTOR - Advanced BLDC Motor Control Simulator         │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ Motor Params │ Load │ Supply │ Control │ Monitoring │ ...│   │
│  ├─────────────────────────────────────────────────────────┤   │
│  │                                                         │   │
│  │  [Control Panel Area]                                   │   │
│  │                                                         │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────┬────────────┬────────────┬──────────┬──────────┐   │
│  │ Start   │ Stop       │ Reset      │ Export   │ Quit     │   │
│  │ (F5)    │ (F6)       │ (F7)       │ (Ctrl+S) │ (Ctrl+Q) │   │
│  └─────────┴────────────┴────────────┴──────────┴──────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Feature 1: Application Name & Branding ⚡

### Before:

```
BLDC Motor Control Simulator - V/f Control System
```

### After:

```
⚡ SPIN DOCTOR - BLDC Motor Control Simulator (v2.0.0)
```

**Everywhere in the app:**

- Window title: ⚡ SPIN DOCTOR
- Speed display: ⚡ Rotor Speed
- Startup log: ⚡ SPIN DOCTOR v2.0.0

---

## Feature 2: Simulation Duration Control

### Location: Control Tab

```
┌─────────────────────────────────────────────┐
│ Simulation Duration                         │
├─────────────────────────────────────────────┤
│                                             │
│  Duration: [10.0] seconds                   │
│            ^    ^ (spinbox: 0-300)          │
│                                             │
│  💡 Tip: Set to 0 seconds for infinite     │
│     simulation (run until Stop pressed)    │
│                                             │
└─────────────────────────────────────────────┘
```

### How It Works:

**Example 1: Finite Duration (10 seconds)**

```
Duration = 10 seconds
       ↓ Start Simulation
Simulation runs exactly 10 seconds
       ↓ Auto-stops
Results ready to view/export
```

**Example 2: Infinite Duration (0 seconds)**

```
Duration = 0 seconds
     ↓ Start Simulation
Simulation runs continuously
     ↓ Press Stop manually
Simulation stops
Results ready to view/export
```

---

## Feature 3: Enhanced Monitoring Tab

### Split-Screen Layout

```
⚡ Monitoring Tab
┌────────────────────────────────────────────────────────┐
│ Real-Time Monitoring                                   │
├──────────────────────┬─────────────────────────────────┤
│                      │                                 │
│  LEFT SIDE:          │  RIGHT SIDE:                    │
│  Status Values       │  Speed Curve Plot               │
│  (Scrollable)        │                                 │
│                      │    Speed (RPM)                  │
│  ⚡ Rotor Speed:     │  1500├─ ─ ─ ─ ─ ─ ─ ─         │
│    1250 RPM          │      │                          │
│                      │  1000├────      ┐               │
│  Angular Velocity:   │      │        /─ ─             │
│    131 rad/s         │   500├────  /                   │
│                      │      │    /                     │
│  Rotor Position:     │      0└  /─────────────────    │
│    6.25 rad          │        0   5   10    Time (s)  │
│                      │                                 │
│  Phase A Current:    │  Hot Pink Line                  │
│    3.16 A            │  Grid + Legend                  │
│                      │  Updates Every 500ms            │
│  Phase B Current:    │                                 │
│   -1.42 A            │                                 │
│                      │                                 │
│  Phase C Current:    │                                 │
│   -1.74 A            │                                 │
│                      │                                 │
│  Torque: 2.34 N·m    │                                 │
│  Back-EMF A: 4.2 V   │                                 │
│  Back-EMF B: -1.1 V  │                                 │
│  Back-EMF C: -3.1 V  │                                 │
│  Sim Time: 3.456 s   │                                 │
│                      │                                 │
└──────────────────────┴─────────────────────────────────┘
```

### 13 Variables Displayed:

1. ⚡ Rotor Speed (RPM)
2. Angular Velocity (rad/s)
3. Rotor Position (rad)
4. Phase A Current (A)
5. Phase B Current (A)
6. Phase C Current (A)
7. Electromagnetic Torque (N·m)
8. Back-EMF Phase A (V)
9. Back-EMF Phase B (V)
10. Back-EMF Phase C (V)
11. d-axis Current (A) [FOC only]
12. q-axis Current (A) [FOC only]
13. Simulation Time (s)

---

## Feature 4: Live Speed Curve Plotting

### What You See:

```
During Simulation:
─────────────────

Speed (RPM)
   2000 │           ╱─ ─ ─ ─ (steady)
        │        ╱─╯
   1500 │     ╱
        │   ╱
   1000 │  ╱ (acceleration)
        │╱
      0 └──────────────────── Time (s)
        0  2  4  6  8  10

→ Updates Every 500ms
→ Shows exact moment motor reaches target speed
→ Detects oscillations or instabilities
```

### Use Case Examples:

**Example 1: Stable Response**

```
Speed (RPM)
   1500├─────────────────────  Desired
        │      ╱
   1000 │     ╱
        │   ╱
    500 │  ╱
        │╱
      0 └────────────────────
        0     5     10    Time

✅ Smooth acceleration, no oscillation
```

**Example 2: Unstable Response**

```
Speed (RPM)
   1500├─┬─┬─┬─┬─ Oscillating!
        │ │ │ │ │
    750 ├─┴─┴─┴─┴─  PID tuning needed
        │
      0 └────────────────────
        0     5     10    Time

⚠️ High gain causing oscillation
```

---

## Feature 5: Standard Menu Bar

### Menu Structure:

```
┌──────────────────────────────────────────────┐
│ File        Tools              Help          │
├──────────────────────────────────────────────┤
│ │                                            │
│ ├─ Export Simulation Data (Ctrl+S)          │
│ │                                            │
│ ├─ Quit (Ctrl+Q)                            │
│                                              │
```

### File Menu

```
┌──────────────────────────────┐
│ File                          │
├──────────────────────────────┤
│ Export Simulation Data        │ → Save CSV + JSON metadata
│                               │
│ Quit (Ctrl+Q)                │ → Close application
└──────────────────────────────┘
```

### Tools Menu

```
┌──────────────────────────────┐
│ Tools                         │
├──────────────────────────────┤
│ Reset Simulation (F7)         │ → Clear data, reset to start
└──────────────────────────────┘
```

### Help Menu

```
┌──────────────────────────────┐
│ Help                          │
├──────────────────────────────┤
│ About SPIN DOCTOR             │ → Show version & features
│                               │
│ Quick Start Guide             │ → 7-step tutorial
└──────────────────────────────┘
```

---

## Feature 6: Keyboard Shortcuts Reference

### Quick Reference Card:

```
╔════════════════════════════════════════════╗
║        SPIN DOCTOR Keyboard Shortcuts      ║
╠════════════════════════════════════════════╣
║ F5          Start Simulation               ║
║ F6          Stop Simulation                ║
║ F7          Reset Simulation               ║
║ Ctrl+S      Export Data                    ║
║ Ctrl+Q      Quit Application               ║
╚════════════════════════════════════════════╝
```

---

## Complete User Workflow

### Workflow A: 10-Second Test

```
Step 1: Configure Motor
├─ Motor Parameters Tab
├─ Set voltage, resistance, inductance, poles
└─ ✓ Done

Step 2: Set Load
├─ Load Tab
├─ Select load type & torque
└─ ✓ Done

Step 3: Choose Control
├─ Control Tab
├─ Select V/f or FOC algorithm
├─ Set speed/current references
└─ ✓ Done

Step 4: Set Duration (KEY!)
├─ Duration: 10 seconds
└─ ✓ Done

Step 5: Start
├─ Click "Start" or press F5
└─ → Simulation runs 10 seconds

Step 6: Monitor
├─ Watch Monitoring Tab
├─ See speed curve build in real-time
├─ After 10s: Auto-stops automatically
└─ ✓ Complete

Step 7: Export
├─ Ctrl+S or File → Export
├─ CSV file with all data
└─ ✓ Analysis ready
```

### Workflow B: Infinite Mode

```
Step 1-3: Configure (same as above)

Step 4: Set Duration (KEY!)
├─ Duration: 0 seconds ← Special!
└─ ✓ Done

Step 5-6: Start & Monitor
├─ Simulation runs continuously
├─ You're in control
├─ Press F6 or "Stop" when ready
└─ → No auto-stop!

Step 7: Analyze & Export
├─ View complete results
├─ Export via Ctrl+S
└─ ✓ Analysis ready
```

---

## Side-by-Side Comparison

### Duration = 10 seconds (Finite)

```
Start (t=0s)
    │
    ├─ Simulation running...
    │  - Speed rising
    │  - Current flowing
    │  - Time: 0-5s
    │
    ├─ Simulation running...
    │  - Speed steady
    │  - Current stable
    │  - Time: 5-10s
    │
End (t=10s)
    └─ AUTO-STOP ✓ (time reached)
```

### Duration = 0 seconds (Infinite)

```
Start (t=instant)
    │
    ├─ Simulation running...
    │  - You watching
    │  - Time: indefinite
    │
    ├─ Simulation running...
    │  - You watching
    │  - When satisfied:
    │
Manual Stop (you press F6)
    └─ Stop Now ← You decide when
```

---

## Tips & Best Practices

### 📊 For Data Analysis:

1. **Short runs:** Use 5-10 seconds, capture transient
2. **Long runs:** Use 60+ seconds, verify steady-state
3. **Debugging:** Use 0 seconds, explore manually

### 📈 For Speed Tuning:

1. **Watch the curve:** Smooth = good tuning
2. **Oscillations:** Reduce gains
3. **Slow response:** Increase gains

### 🎯 For Commissioning:

1. **Capture startup:** Speed curve shows startup behavior
2. **Compare:** Run multiple tests, export, compare in Excel
3. **Validate:** Match sim results with hardware measurements

### 💾 For Documentation:

1. **Export often:** Keep CSV records
2. **Name clearly:** Use relevant filenames
3. **Screenshot:** Capture monitoring displays

---

## What's Improved vs v1.0.0

| Feature              | v1.0.0         | v2.0.0                     |
| -------------------- | -------------- | -------------------------- |
| App Name             | Generic        | ⚡ SPIN DOCTOR             |
| Version Display      | None           | v2.0.0 (prominent)         |
| Duration Control     | N/A            | 0=infinite, >0=finite      |
| Monitoring Variables | 11             | 13 (added pos, time)       |
| Speed Display        | Numerical only | + Live graph               |
| Menu Bar             | Buttons only   | Full File/Tools/Help       |
| Help                 | Inline tips    | About + Quick Start        |
| Keyboard Shortcuts   | Partial        | Complete (F5-F7, Ctrl+S/Q) |

---

## Troubleshooting

### Q: Why did my simulation stop after exactly 10 seconds?

**A:** Check Control Tab → Duration. It was set to 10 seconds. Set to 0 for infinite.

### Q: The speed curve isn't updating?

**A:** It updates every 10 samples (~500ms). Refresh happens less frequently to save CPU.

### Q: Where's my exported data?

**A:** Check: `/logs/` folder in project directory. Files named `simulation_YYYYMMDD_HHMMSS.csv`

### Q: How do I see more monitoring variables?

**A:** All 13 are already displayed! Scroll down in the left panel if needed.

### Q: Can I customize the speed curve color?

**A:** Yes! Edit line 1178 in `src/ui/main_window.py`, change `color='#FF6B9D'` to your preferred hex color.

---

## 🎉 You're Ready to Use SPIN DOCTOR v2.0.0!

```
Launch: python main.py

Try:
1. Set Duration = 5 seconds
2. Click Start (F5)
3. Watch speed curve auto-update
4. Simulation auto-stops after 5s
5. Export results (Ctrl+S)

Done! 🚀⚡
```

---

**Version:** 2.0.0  
**Features:** 6/6 Implemented  
**Status:** ✅ Production Ready  
**Last Updated:** March 4, 2026
