# Presentation story, slide by slide (rewritten 2026-08-26)

A narrative walkthrough of the deck's flow — the bird's-eye view, for
getting the whole story in one pass before diving into per-slide detail
(`points.txt`/`notes.txt` in each `slide_NN_*/` folder). Not slide content
itself.

This deck replaces the prior "belief-divergence fault detection" story
entirely (retired slides preserved in `_retired_pre_pivot/` for history).
The core message: **Layer 1 (a calibrated, validated uncertainty signal)
is close to done. Layer 2 (translating that signal into physical
consequence) is the real remaining thesis work, and this deck exists to
show the concrete path from one to the other.**

**01 — Title.** Same core framing as every prior presentation — AI safety
in AVs via digital twins. The subtitle is what changed again: calibrated
uncertainty and consequence estimation, not fault detection or belief
divergence. Signals a maturation of the claim, not a change of subject.

**02 — Motivation.** Opens with the same move as before: our prior T-ITS
paper proved the twin CAN detect faults accurately — but accuracy alone
doesn't answer whether a prediction's confidence can be trusted. Reframes
"can it detect" into "does it know what it doesn't know, and does that
self-knowledge respond correctly when something goes wrong."

**03 — Literature gap.** Two traditions that don't talk to each other:
uncertainty quantification (treats confidence as an alarm to threshold)
and risk-aware planning (assumes its own predictions are correct). Names
the closest real prior work (Reuter et al., ITSC 2026) precisely and
states exactly how this differs. Reports an independent literature-review
agent's verdict: the specific combination here wasn't found anywhere.

**04 — Problem statement.** The three-layer architecture (Detect /
Consequence / Respond) and the thesis statement. States plainly what's
banked (Layer 1, this update) vs. what's the real remaining claim (Layer
2). Introduces the decision to split into two papers rather than one,
and why that's not a hedge.

**05 — Framework.** The mechanism, replacing belief-divergence entirely:
calibrated interval → bootstrap counterfactual futures → reachability
margin → calibrated P(violation). Explains precisely why this is the one
path that doesn't drop the thread between the two literature traditions
from slide 3.

**06 — Digital twin (architecture).** The ST-GAT model, and — importantly
— the abandoned approach that came before it: jointly-trained
distributional heads, which measurably failed to calibrate. States
plainly that the switch to conformal calibration was about calibration
quality, not point accuracy (which is actually mixed). A real "we tried
the obvious thing, here's why it didn't work" narrative beat.

**07 — Experimental platform.** AWSIM + Autoware, unchanged from prior
work. Notes that the same zone geometry built for fault-injection gating
turned out to be exactly the right taxonomy for this cycle's calibration
auditing too.

**08 — Data collection.** What's actually on disk: the 26-goal nominal
baseline (with the honest caveat that only 7 trials are truly held out —
load-bearing, not polish) and the fault taxonomy (IMU state-estimation
corruption vs. TL perception spoofing, 8 campaigns + 3 new fixed-severity
ones). Previews the IMU-vs-TL failure-signature distinction that pays off
on slide 13.

**09 — Fault plans and impacts.** The message-level verification step:
did each fault mechanism actually change the raw signal it targets, on
the real driven trajectory — necessary but not sufficient, sets up
slide 13's deeper question.

**10 — Calibration baseline, and the gap found.** Shows the good result
first (reliability diagram, efficiency curve — both real, both
validated), then the concrete example (the sharpest turn in the dataset)
that surfaced a real, hidden per-scenario failure the aggregate numbers
couldn't show. The "before" half of this update's central before/after
story.

**11 — Finding and fixing the gap.** The geometry-grounded audit
quantifies the hidden gap (75.6% intersection coverage inside a 90%
pooled number), then the fix: retrain the model itself, not just widen
the interval — a real methodological argument, not just an engineering
choice. Shows the SAME turn window as slide 10, after the fix — the "after"
half of the story, and the deck's most persuasive visual.

**12 — Conditional calibration, compared honestly.** Two ways to make the
calibrated interval scenario-aware: discrete (Mondrian) vs. continuous
(the model's own learned scene similarity). Reports the real, quantified
trade-offs of each — including that Mondrian is NOT a free efficiency win
once properly weighted, a finding caught and corrected before being
oversold.

**13 — Layer 1 meets real faults.** The fault-validation payload: two
qualitatively different fault signatures (IMU compounding vs. TL
contained), and a likely-novel finding that the most diagnostic feature
isn't the obvious one. Closes with a first-look severity dose-response
result from data nobody had analyzed this way before, including an
honestly-reported confound that directly shaped the next data collection
round's design.

**14 — Interpretation.** Steps back to state plainly what's solid (the
methodology, the audit-fix cycle, the fault validation) and what needs to
be treated as a real limitation (calibration-set size, the sweep's pilot
status, Mondrian's real cost) — not glossed over. Reports the independent
novelty verdict's exact conditions. Closes by recommending Layer 2 as the
next real payoff, not further Layer 1 polish.

**15 — Next steps.** The concrete, already-planned path to closing Paper
1 (the redesigned severity sweep, matched-pair data collection) running
in parallel with starting Layer 2 (the reachability-margin redesign, the
target result shape — four artifacts in priority order). Ends on two real
questions for the advisor: the two-paper split, and whether NPC/traffic-
density belongs in Layer 2's scope.
