# Student Guide: Using AI to Help You Program (Not Program For You)

Welcome to FRC 5010 programming! This guide is for **you, the student**, on how to use AI
tools like Claude Code to actually become a better programmer — instead of accidentally
becoming someone who can only program when an AI is open.

It's a longer read than the project's [CLAUDE.md](../CLAUDE.md), and it's worth doing once at
the start of the season.

> **TL;DR** — AI is a tutor and a pair-programmer, not an autocomplete. If you can't explain
> why your code works, you don't actually have working code — you have a time bomb that will
> blow up at competition.

---

## Table of contents

1. [The mindset](#1-the-mindset)
2. [The rules](#2-the-rules)
3. [What AI is great at](#3-what-ai-is-great-at)
4. [What AI is bad at](#4-what-ai-is-bad-at)
5. [How to write a good prompt](#5-how-to-write-a-good-prompt)
6. [The "explain-first" workflow](#6-the-explain-first-workflow)
7. [What to do when you're stuck](#7-what-to-do-when-youre-stuck)
8. [Glossary: FRC terms you'll hear](#8-glossary-frc-terms-youll-hear)
9. [Where to learn more](#9-where-to-learn-more)

---

## 1. The mindset

There are two ways to use AI for programming:

**The shortcut way (don't do this):**

> "Write me a subsystem that controls the intake."
> *AI dumps 80 lines.*
> *You paste it. It compiles. You move on.*

You just lost the entire reason you joined the programming team. At competition, when
something breaks, you'll have no idea how to fix it. You'll feel helpless. Your team will lose
because you don't actually understand the robot you "wrote."

**The learning way (do this):**

> "I'm trying to build an intake subsystem. The intake has one motor that spins a roller, and
> a sensor that detects when a game piece is in. I read the WPILib command-based docs but I'm
> not sure how to wire the sensor reading into a command. Can you walk me through how a
> subsystem like this should be structured?"

The AI now teaches you a pattern. You write the code yourself, asking for help on each step.
At the end, you understand every line. At competition, when the sensor wire comes loose,
you'll know exactly what to debug.

**The goal of using AI is to become someone who needs AI less over time, not more.**

---

## 2. The rules

These are the team's rules for using AI assistants like Claude Code in this codebase. Mentors
expect you to follow them.

### Rule 1: Explain before you ask for code

Before asking for code, you must be able to say:

1. **What** you're trying to accomplish (in plain English).
2. **Why** you're trying to accomplish it (what bigger goal does it serve?).
3. **What** you've already tried or considered.

If you can't fill in those three blanks, you're not ready to ask for code yet — you're ready
to ask for an explanation.

### Rule 2: Never paste code you don't understand

If the AI generates code and you don't understand a line, **stop and ask about that line**
before continuing. "What does this do?" is a great question. "Why this approach instead of
[some other approach]?" is even better.

### Rule 3: You must be able to summarize what was written

After a session where you and the AI changed code, you should be able to explain to a
teammate (or a rubber duck on your desk):

- What changed.
- Why it changed.
- What would break if you deleted the change.

If you can't, you're not done — you're just on pause.

### Rule 4: Never deploy code to the real robot without simulating first

Always test in **simulation** (`./gradlew simulateJava` or the WPILib *Simulate Robot Code*
command in VSCode) before pushing code to the actual roboRIO. Robots are expensive. Bumpers
catch fire when the wrong motor turns on. Don't be the reason.

### Rule 5: Never skip the format check or the tests

If `./gradlew build` fails because of formatting or a test, fix the actual problem. Don't ask
the AI to bypass it. The team's hooks exist to keep the codebase consistent and working.

### Rule 6: Never let AI commit, push, or deploy on your behalf without you confirming

When Claude offers to run `git commit`, `git push`, or `./gradlew deploy`, **read what it's
about to do** and approve it consciously. These actions affect the whole team.

### Rule 7: Ask a mentor when you're unsure

If something feels off — the AI is suggesting changes you don't understand, or you're not
sure if a pattern matches the project — pause and ask a mentor. Ten minutes of mentor time
saves ten hours of debugging.

---

## 3. What AI is great at

Lean on AI for these — it's faster and clearer than searching, and a mentor will be glad you
didn't take their time on them:

- **Explaining unfamiliar code.** "Walk me through what this method does, line by line."
- **Explaining unfamiliar concepts.** "What is a PID controller and when would I use one?"
- **Generating examples.** "Show me a small example of a command that runs for 2 seconds and
  then ends."
- **Suggesting names.** "I have a method that picks the closest target and returns its angle —
  what's a good name?"
- **Debugging error messages.** "I'm getting this stack trace, what does it usually mean?"
- **Refactoring small chunks.** "Can this if/else chain be cleaner?"
- **Reviewing code you wrote.** "I just wrote this method. Anything I'm missing? Any edge
  cases I forgot?"
- **Writing tests for code you already wrote.**

---

## 4. What AI is bad at

Do *not* trust AI alone for these:

- **Knowing what's actually on the robot.** AI can't see your wiring. Always verify CAN IDs,
  port numbers, and gear ratios with the build team — don't trust whatever the AI guesses.
- **The latest WPILib changes.** WPILib evolves every year and AI training data lags. If the
  AI suggests an API that doesn't exist, check the WPILib 2026 docs.
- **Knowing your team's conventions.** AI doesn't know that team 5010 prefers `frc.robot.rebuilt.*`
  over the deprecated `org.frc5010.common.*` unless something (like CLAUDE.md) tells it.
- **Physics and tuning.** AI can suggest PID values, but they're guesses. Real tuning happens
  on the real robot, slowly, with a person watching.
- **Whether your test is meaningful.** AI will happily write a test that passes but tests
  nothing.
- **Judgement calls.** Should this be one class or two? Should we add a feature or cut scope?
  These are *your* calls (and your mentors'), not the AI's.

---

## 5. How to write a good prompt

A bad prompt:

> *"intake not working"*

A good prompt:

> *"I'm working on the Intake subsystem in `subsystems/intake/Intake.java`. When I press the
> A button on the driver controller, the intake motor should spin forward. Right now nothing
> happens. I added the button binding in `IntakeCommands.configureButtonBindings()`. I checked
> in sim and the button press is being detected (I see it on the dashboard), but the motor
> output stays at 0. What should I check next?"*

The good prompt has:

- **Where** you are (file, subsystem).
- **What** you expected to happen.
- **What** actually happened.
- **What** you've already verified.
- **A specific question.**

A prompt like that gets you a focused, useful answer. The bad one gets you a generic checklist
you've already done.

---

## 6. The "explain-first" workflow

This is the workflow your mentors expect when you sit down with an AI assistant:

1. **State the goal** in plain English. ("I want the launcher to spin up when the operator
   holds the right trigger, and stop when they release it.")
2. **Ask the AI to describe the approach** without writing code yet. ("Can you describe how
   I'd structure that, using the command-based pattern we use here?")
3. **Confirm you understand the approach.** Ask follow-up questions. ("Why a `whileTrue`
   instead of `onTrue`?")
4. **Try writing it yourself first.** Even a rough draft. Then share it with the AI.
5. **Ask for feedback on your draft**, not a rewrite. ("Here's what I wrote. Does the
   structure look right? What am I missing?")
6. **Make the fixes yourself**, asking for clarification on anything you don't get.
7. **Test in simulation.** Watch the dashboard. Make sure the behavior matches what you
   expected.
8. **Summarize what you did** in your own words before closing the AI window. If you can't
   summarize it, you're not done.

This feels slower than just asking for code. It is slower — by a factor of maybe 2x. But what
you learn in those 2x sessions makes you 10x faster a month from now, and 100x more useful at
competition.

---

## 7. What to do when you're stuck

Try things in this order before pinging a mentor:

1. **Read the error message carefully.** Java errors are wordy but specific. The line number
   matters.
2. **Look at recently changed files.** `git status` and `git diff` show what you touched.
3. **Ask the AI to explain the error.** Paste the full stack trace, not just the top line.
4. **Search the WPILib docs.** They're at <https://docs.wpilib.org/>.
5. **Look for a similar pattern already in the codebase.** Use VSCode's *Go to Symbol* or
   *Find in Files*. Most things you want to do, someone on the team has already done somewhere.
6. **Step away for 5 minutes.** Seriously. Walk to the water fountain. The answer often
   appears.
7. **Ask a teammate** who's been on the team longer.
8. **Then ask a mentor.** Show them what you've tried.

---

## 8. Glossary: FRC terms you'll hear

| Term | What it means |
| --- | --- |
| **WPILib** | The official robot programming library from FIRST. Provides the base classes (`Robot`, `Subsystem`, `Command`) and hardware wrappers. |
| **Subsystem** | A class representing one mechanism on the robot. Has motors, sensors, and a `periodic()` method that runs every 20 ms. |
| **Command** | A class representing one *action* the robot can take (spin up, drive forward, fire). Commands are scheduled against subsystems. |
| **Command-based** | The architectural style WPILib uses: code is organized as subsystems + commands instead of one big loop. |
| **Periodic** | A method that runs every robot loop (every 20 ms). `robotPeriodic`, `teleopPeriodic`, `subsystem.periodic()`, etc. |
| **Scheduler** | The thing that runs commands. You don't usually call it — `CommandScheduler.getInstance().run()` is called for you in `robotPeriodic`. |
| **AdvantageKit** | A logging library by team 6328 that records every input/output of the robot so you can replay matches in sim. It's why we use `LoggedRobot` and the `IO` interface pattern. |
| **IO layer** | The split between a subsystem's logic (e.g. `Intake.java`) and the code that talks to hardware (`IntakeIOReal.java`, `IntakeIOSim.java`). Lets us run the robot on a laptop without changing the logic. |
| **roboRIO** | The computer on the robot that runs your code. We deploy code to it over a network. |
| **CAN bus** | The network the roboRIO uses to talk to motor controllers and sensors. Every device has a CAN ID. |
| **PathPlanner** | A tool for designing autonomous routines as paths on a field, used by our `AutoCommands`. |
| **PID** | A control algorithm (Proportional, Integral, Derivative) for making a motor reach a target — like spinning a flywheel to exactly 3000 RPM. |
| **Pose** | The robot's position and rotation on the field. Usually `Pose2d(x, y, rotation)`. |
| **Kinematics** | The math that converts "I want to drive at X m/s, turning at Y rad/s" into individual wheel/swerve-module speeds. |
| **SmartDashboard / Elastic** | Dashboards we use to show data to the drive team during a match. |
| **Driver Station** | The laptop program that talks to the robot — sends joystick data, shows logs, enables/disables. |
| **Spotless** | The code formatter that runs before every compile. Don't fight it. |
| **Lombok** | An annotation library that auto-generates boilerplate (getters, setters). When you see `@Getter`, that's Lombok. |

---

## 9. Where to learn more

In rough order of usefulness for a new student:

- **WPILib Zero to Robot:** <https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html>
- **WPILib Command-Based Guide:** <https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html>
- **AdvantageKit Docs:** <https://docs.advantagekit.org/>
- **PathPlanner Docs:** <https://pathplanner.dev/home.html>
- **Project-specific docs:** [docs/](.) — Javadoc guidelines, unit enum pattern, JSON schemas.
- **Our team Discord / Slack** (ask your mentor for the link).
- **Chief Delphi** (community forum): <https://www.chiefdelphi.com/>

---

*Build season is short. Use AI to learn faster, not to skip the learning. Future-you, at 11pm
the night before week-zero, will thank present-you for actually understanding the code.*
