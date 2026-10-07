# Fault Literature Review — Camera & IMU (starting points, 2026-07-22)

**Status:** First pass, done to kick off the exploratory fault-reaction study (see
`TODO.md`). Not exhaustive — flagged papers are starting points for a deeper read next
session, prioritized by how directly they can inform realistic fault *parameters* for
`experiments/lib/fault_injector.py`, not just general survey background.

**Why this matters now:** the T-ITS paper (`Digital_Twins_as_Predictive_Models_for_Real-Time_Probabilistic_Risk_Assessment_of_Autonomous_Vehicles.pdf`,
project root) picked its Camera/IMU/LiDAR fault models somewhat arbitrarily —
Gaussian pixel noise (σ=10) for camera, random bias within a unit sphere for IMU —
justified only by citing Matos et al.'s survey as evidence these sensor types matter,
not that *these specific fault shapes* are realistic. The goal here is to ground the
next round of fault injection (IMU + Camera only, LiDAR deferred) in what real sensor
failure modes actually look like.

---

## Already-cited anchors (from the paper's own bibliography — highest priority re-reads)

- **Matos, Bernardino, Duraes, Cunha, "A Survey on Sensor Failures in Autonomous
  Vehicles: Challenges and Solutions," Sensors 24(16), 2024.**
  https://www.mdpi.com/1424-8220/24/16/5108 / PMC: https://pmc.ncbi.nlm.nih.gov/articles/PMC11360603/
  108-publication survey; the paper's sole justification for choosing Camera/IMU/LiDAR
  as fault targets. Worth a full re-read specifically for the *taxonomy* of camera and
  IMU failure modes it catalogs (the T-ITS paper only cites it at the "these sensors
  matter" level, not for fault-shape realism).
- **Zhang, Carballo, Yang, Takeda, "Perception and sensing for autonomous vehicles
  under adverse weather conditions: A survey," ISPRS J. Photogramm. Remote Sens. 196,
  2023.** — cited in the paper specifically for camera vulnerability to dirt/debris/
  condensation/lighting. Good source for occlusion realism.
- **Jha, Banerjee, Cyriac, Kalbarczyk, Iyer, "AVFI: Fault Injection for Autonomous
  Vehicles," DSN-W 2018.** https://arxiv.org/pdf/1907.01038 — the fault-injection
  methodology framework the paper leans on. Check for IMU/camera fault magnitude
  precedent.

## Camera fault / occlusion realism (new, this session)

The T-ITS paper's camera fault is uniform Gaussian pixel noise (σ=10) — a stand-in for
"the camera got worse," not modeled on any specific failure mechanism. This repo's
current camera-adjacent fault (`fault_injector.py`'s TL_* modes) operates one level
downstream, on the traffic-light *classification output*, not raw pixels — closer to
"perception got worse" than "camera got worse." Papers below are useful for deciding
whether to (a) keep faulting at the TL-signal level but make the degradation curve
realistic, or (b) add a genuine image-level fault (soiling/occlusion mask on the
camera topic) closer to the paper's own approach:

- **"Let's Get Dirty: GAN Based Data Augmentation for Camera Lens Soiling Detection in
  Autonomous Driving"** (WoodScape team). https://arxiv.org/pdf/1912.02249 — real
  soiling patterns (mud, water, dust) captured on a physical rig; the de-facto
  reference dataset (WoodScape/SoilingNet) for what lens contamination actually looks
  like, as opposed to uniform noise.
- **"Effect of Droplet Contamination on Camera Lens Surfaces: Degradation of Image
  Quality and Object Detection Performance,"** Applied Sciences 15(5), 2025.
  https://www.mdpi.com/2076-3417/15/5/2690 — quantifies MTF50 (sharpness) dropping up
  to 80% at 10μL droplet volume, with a direct link to detection accuracy. Gives an
  actual numeric fault-severity curve to anchor tier choices against, rather than
  picking σ arbitrarily.
- **"Performance Degradation of Object Detection Neural Networks Under Natural Visual
  Contamination in Autonomous Driving,"** Computers 15(4), 2024.
  https://doi.org/10.3390/computers15040254 — directly measures detector accuracy loss
  under realistic contamination (not synthetic noise).
- **"Occluded nuScenes: A Multi-Sensor Dataset for Evaluating Perception Robustness in
  Automated Driving,"** 2025. https://arxiv.org/pdf/2510.18552 — synthetic occlusion
  types (dirt, waterblur, WoodScape-style soiling, scratches) applied to a standard
  benchmark; a template for how to parameterize severity tiers the way this repo
  already does for TL faults (S1-S4).
- **"Evaluating the Impact of Weather-Induced Sensor Occlusion on BEVFusion for 3D
  Object Detection,"** 2025. https://arxiv.org/html/2511.04347v1 — camera-only models
  show sharp drops even under *moderate* occlusion (LiDAR much more robust to
  equivalent point dropout) — relevant if we ever want to argue camera-fault severity
  tiers should be steeper than IMU ones.
- **Traffic-light-specific:** general TL detection literature confirms glare, motion
  blur, and dark/cloudy lighting are the dominant real-world TL misclassification
  causes (vs. sustained/complete occlusion causing outright detection loss) — read
  before finalizing the "camera fault" story, since Traffic Light Status Flag was the
  *single most important feature* (29.7%) in the T-ITS paper's fault classifier. That
  makes the existing `fault_injector.py` TL fault modes (`tl_confidence`,
  `tl_oscillate`, `tl_unknown`, `tl_blackout`) more central to a "camera fault" study
  than they might first appear — worth explicitly deciding whether TL-signal faults
  alone represent "camera fault" for this study, or whether a raw-pixel/image-level
  fault should be added alongside them.

## IMU fault realism (new, this session)

This repo's existing IMU fault (`fault_injector.py`'s `imu_bias` mode) injects a
constant gyro bias (0.05–0.60 rad/s across 4 tiers) — reasonably well-grounded already
(matches the "bias/drift" failure mode both the T-ITS paper and general IMU literature
identify as dominant), but the fault_injector.py docstring notes `accel_bias_ms2` is
currently a **no-op** in the pipeline (gyro_odometer doesn't consume linear
acceleration) — worth revisiting given IMU literature treats accelerometer bias as an
equally standard fault mode, not just gyro bias.

- **General IMU fault taxonomy:** bias (turn-on offset + long-term instability), drift
  (bias instability + angular/velocity random walk), and scale-factor errors are the
  three standard categories across the literature — this repo's tiers only cover
  gyro bias/drift, not scale-factor faults. Possible additional fault mode to consider.
- **"Fault Detection and Classification of Aerospace Sensors using a VGG16-based Deep
  Neural Network,"** 2022. https://arxiv.org/pdf/2207.13267 — not AV-specific, but a
  clean methodological analog: injects bias, drift, and stuck-at faults into IMU-class
  sensors for a learned fault classifier, structurally similar to what this project's
  ST-GAT + residual approach is doing.
- **Gyroscope bias → lateral drift mechanism:** a rate-gyro bias error causes the
  vehicle to believe it's centered in-lane while actually veering — already the
  intuition documented in `fault_injector.py`'s comments; general robotics literature
  confirms this is the standard real-world failure signature, supporting the choice to
  bias `angular_velocity.z` specifically as the highest-leverage IMU fault channel.
- **CARLA+Autoware fault-injection framework** (found via search, not yet read in
  detail) — a directly comparable simulation-based fault injection framework combining
  LiDAR/GNSS/IMU faults on the *same Autoware stack* this project uses, just on CARLA
  instead of AWSIM. Worth reading for methodology comparison (how they parameterize
  IMU fault magnitude) even though it doesn't cover camera.

## Recent (2025) multi-sensor fault-injection frameworks — methodology comparisons

- **"Testing the Fault-Tolerance of Multi-Sensor Fusion Perception in Autonomous
  Driving Systems,"** ISSTA 2025. https://arxiv.org/pdf/2504.13420 — validated on a
  real Apollo 6.0 vehicle, not just simulation; useful for how they define fault
  "severity" in a way that's traceable to real-world consequences (their framing of
  safety violations, not just detection-accuracy loss).
- **"Injecting Hallucinations in Autonomous Vehicles: A Component-Agnostic Safety
  Evaluation Framework,"** 2025. https://arxiv.org/pdf/2510.07749 — explicitly argues
  most existing fault-injection work is siloed per-sensor (exactly what this repo's
  `fault_injector.py` currently is: TL-fault and IMU-fault as two independent relay
  nodes) — relevant if a future direction is testing *combined* camera+IMU faults
  rather than one at a time.

## Open questions to resolve next session (not answered here)

1. Does "camera fault" for this exploratory study mean the existing TL-signal-level
   faults, a new raw-image-level fault, or both? The TL literature above suggests the
   existing infra is more defensible than it looks (TL Status Flag was the dominant
   fault-detection feature in the prior paper), but it's not literally a "camera"
   fault in the sensor sense.
2. Should `accel_bias_ms2` be wired up to actually affect the pipeline (currently a
   documented no-op), given IMU literature treats accelerometer bias as a standard
   fault mode alongside gyro bias?
3. Do we want severity tiers grounded in a *measured* degradation curve (e.g. the
   MTF50-vs-droplet-volume result above) rather than the round numbers currently used
   for TL/IMU tiers?
