# From a script to a "task"

This folder holds the **tasks** for the redesigned (PyChrono 10.0) version of ChronoBench. This note
explains, in plain terms, what a "task" actually is, so anyone can add one correctly.

## The big idea

ChronoBench tests how well an AI can write a simulation script for the Chrono physics engine. But a
script that merely *runs* is not enough to grade an AI fairly. We need a script that runs **and proves
it did the right thing**, like a science experiment that not only happens but also measures a result
you can check against the textbook answer.

So a **task** is a small, self-checking virtual experiment. Think of it like a graded lab: there is an
assignment, a correct answer, and an automatic way to check the work.

## What a task is made of

To turn an ordinary script into a task, you give it these parts (the `pendulum/` folder is the
simplest complete example):

1. **A prompt** (`input1.txt`): the plain-English assignment the AI receives. Example: "Build a
   pendulum 1 meter long under Earth gravity, release it from a small angle, run for 5 seconds, and
   report its swing period."
2. **Exact numbers:** the assignment pins down the specifics (length, gravity, mass, how long to run,
   any random seed). Without fixed numbers, there is no single answer to check.
3. **A reference solution** (`truth1.py`): an expert-written, correct script that we have actually run
   and confirmed works on PyChrono 10.0. This is the "answer key."
4. **A measured result:** the script must *report a number* we can check, saved to a file (like
   `out.csv`) plus a short result line (like `{"period_est": 2.01}`). A simulation that only pops up
   an animation is not measurable, so it cannot be a task. This is the step that turns a "demo" into
   an "experiment."
5. **A check** (`contract.json`): a small settings file that says how to run the script, what
   capability it must include, and, most importantly, the **physics answer it must match** (for the
   pendulum, the period must be close to the textbook value, 2*pi*sqrt(L/g), within a tolerance).

## The graduation test

A script only *becomes* a task when its own answer key passes its own check. We run the reference
through the automatic grader (`../scoring/judge_v2.py`), which asks three things:

1. **Does it run?** No crashes, and it finishes within the time limit.
2. **Does it include the required pieces?** For example, it actually built the joint the task is about.
3. **Does the measured result match the physics?** For example, the reported period is correct.

If all three pass, the task is marked **verified** and is ready to use. A human expert also gives it a
final look, because "it ran and the number looks reasonable" is not always the same as "it is truly
correct."

## One more rule

A task is defined by **what skill it tests**, not by the story it tells. Two scripts that test the
same skill (say, two slightly different cars driving on flat ground) count as **one** task, not two.
This keeps the benchmark sharp and avoids asking the same question over and over.

## Where to look

- `pendulum/` -- the simplest complete example (prompt-style contract, answer-key script, and check).
- `STATUS.md` -- the live list of tasks and whether each is verified yet.
