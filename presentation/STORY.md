# Presentation story, slide by slide

A narrative walkthrough of the deck's flow, for reviewing structure before diving
into per-slide detail (`points.txt`/`notes.txt` in each `slide_NN_*/` folder).
Not slide content itself — this is the "why does slide N lead to slide N+1" layer.

**01 — Title.** Same core framing as the last presentation: AI safety in AVs via
digital twins. Subtitle signals what's new this cycle (calibrated confidence +
lead time), not a change of subject.

**02 — Motivation.** Opens with a deliberately narrow claim: our prior T-ITS
paper proved the twin CAN detect faults, accurately. This slide argues that
accuracy alone doesn't answer whether the detector should be *trusted* — sets
up the reframe from "can it detect" to "can it be verified."

**03 — Literature gap.** Places the work in context: fault detection (mature,
including our own prior paper) and trajectory prediction (mature, fast-moving)
are both crowded. The underserved combination — a mechanistic map-grounded
signal + a distribution-free calibration guarantee + an actual lead-time
measurement — is the gap. Sets up why this is a genuine contribution, not a
rerun of the paper.

**04 — Problem statement.** The thesis statement itself. Explicitly separates
what's banked (detection, from the paper) from what's new (calibrated
confidence, lead time, severity). Introduces the two-arm design and flags
honestly that Arm B isn't collected yet — a limitation stated up front, not
discovered later.

**05 — Framework.** The mechanism: belief divergence under a map-grounded
prior, with the "epistemic stance" language explaining *why* this counts as a
legitimate check and not the model grading its own homework. This is the
slide flagged for a clarity pass (see your question 4) — right now it risks
implying the whole model is map-anchored when really only the TL channel is.

**06 — Digital twin (architecture).** The concrete ST-GAT model: 14 input
features, the two new output heads added this session, and the two real bugs
fixed (entity-collapse, missing heads). This is where "the mechanism from
slide 5 is actually implemented" lands.

**07 — Experimental platform.** AWSIM + Autoware, largely unchanged from
before — establishes this runs on the real production stack, not a toy
planner. Updates the goal count (26, not 3) and drops the retired obstacle-
placement framing.

**08 — Data collection.** What's actually been collected: the 26-goal nominal
baseline and the 8 fault campaigns, plus this session's zone-coverage
expansion (3→26 goals, analysis-only, the thing that unlocked slide 11's
tightened guarantee). Also plants the negative-evidence-subtype distinction
(tl_s3/s4 vs. tl_s2/ramp) that pays off in slide 11.

**09 — Fault plans and impacts.** The verification step: did each fault
mechanism actually change the signal it targets, on the actual driven
trajectory. (Flagged for regeneration — see your question 5; the example
plots were stale, generated against a superseded data collection run.)

**10 — Model predictions.** What the trained model's output actually looks
like: a real predicted trajectory + uncertainty on the map, plus the other
predicted channels. This is the "does the model work at all" sanity check
before slide 11 asks "does it work under faults." (Map sizing fixed this
session — see your question 6.)

**11 — How the model reacts to faults.** The payload slide: discriminability
findings, the calibration-tightening result, and detection-rate/lead-time
numbers. Framed deliberately as supporting evidence, not a single headline
number — this is the slide most directly shaped by your pushback on lead time
not being the centerpiece. (Fatal-moment definition flagged for rework — see
your question 7; current lead-time numbers are likely inflated.)

**12 — Interpretation.** Steps back from the numbers to state directly what
they do and don't support — specifically, that the discriminability finding is
a refinement of an existing claim, not a new contribution, and that lead time
should stay a supporting axis. This slide exists specifically to pre-empt
over-claiming from slide 11's results.

**13 — Next steps.** What's left given no new data collection for now: the
severity axis, the CUSUM-search extension, the open TL-fault data gap, and
the active-rectification idea as an explicit scope question for you, not a
plan. Ends on two real questions for you, not rhetorical ones.

---

## Open items from your last review (not yet reflected above)

- Title: fixed (back to the AI-safety-in-AVs framing, subtitle carries the new angle).
- Slide 5's "epistemic stance" language: needs a precision pass — see the
  separate answer to your question 4.
- Slide 9's example plots: being regenerated against current (non-stale) trial
  data.
- Slide 10's example plot: fixed (map now scales to the window's own extent).
- Slide 11's lead-time numbers: likely need a redefined fatal-moment anchor
  before they're trustworthy — see the separate answer to your question 7.
- Whether lead time is paper-shaped on its own, and whether "closing the loop"
  should shape the current experimental design: see the separate answer to
  your question 3 — recommend NOT redesigning the current setup for it.

---

## 2026-08-06 update — direction reframe + a real calibration finding

Bigger than a slide tweak, flagging here rather than silently editing the
walkthrough above. Direction reframed away from binary fault detection
toward a continuous decision-support signal ("operational under
degradation" — see CLAUDE.md/TODO.md). Following that reframe's own
prescribed order (ground the model's credibility before anything else)
surfaced a real, traced problem: the model's predicted uncertainty is
measurably miscalibrated (leptokurtic residuals — Anderson-Darling
decisively rejects normality for all 6 Gaussian heads — and non-widening
variance across the prediction horizon for 5 of 6), and that's the direct
mechanistic cause of the new SPRT signal saturating under pure nominal
noise. Full writeup: `docs/research_notes/trust_and_signal_behavior_2026-08-06.md`.

Slides 10 and 11's notes.txt/points.txt are updated in place with the new
figures and the corrected story — 10 gets the calibration finding, 11 gets
both the SPRT-replaces-CUSUM code change and an explicit "don't present the
old detection-rate/lead-time numbers as current" flag. Slide 11 in
particular reads as two different stories stitched together now (the old
discriminability/lead-time payload + the new calibration-cascade story) —
worth deciding whether it should split into two slides once the
calibration fix is in, not attempted here since that's a real editorial
call, not a mechanical update.
