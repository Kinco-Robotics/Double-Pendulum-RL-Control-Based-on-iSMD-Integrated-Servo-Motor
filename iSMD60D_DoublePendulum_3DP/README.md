# Delivery Notes

## 1. Project Information

- **Version**: v1.0.0
- **Delivery date**: 2026-08-04
- **Contents**: Kinco Motor Control SDK, double pendulum simulation & deployment project, hardware installation manual, and demonstration videos.

---

## 2. Directory Structure

```text
├── README.md                                           # Delivery notes
├── Double_Pendulum_Arm_Kit_Installation_Manual_EN.pdf  # Hardware installation manual
├── Kinco-Motion-Control-SDK-main/                      # Kinco Motor Control SDK: Communicate with drives via RS232/RS485, PCAN, or CxCAN.
│   ├── libs/                                           # Dynamic libraries (.dll / .so / .dylib)
│   ├── examples/                                       # Example programs
│   │   └── example.py                                  # Python API examples
│   └── README.md                                       # SDK Setup and Usage
├── iSMD60D_DoublePendulum_3DP_delivery/                # Double pendulum simulation and deployment project
│   ├── scripts/                                        # Training / playback / deployment entry points
│   ├── source/                                         # Isaac Lab extension and hardware driver
│   ├── models/                                         # USD / USDZ robot assets
│   ├── logs/                                           # Training logs and checkpoints
│   └── ...                                             # Remaining project files
└── Media/                                              # Demonstration video
```
