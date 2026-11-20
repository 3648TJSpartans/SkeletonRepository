# 3648 Skeleton Repository

[![CI Pipeline](https://github.com/3648TJSpartans/SkeletonRepository/actions/workflows/ci.yml/badge.svg)](https://github.com/3648TJSpartans/SkeletonRepository/actions/workflows/ci.yml)
[![Last Commit](https://img.shields.io/github/last-commit/3648TJSpartans/SkeletonRepository)](https://github.com/3648TJSpartans/SkeletonRepository/commits)
[![Commit Activity](https://img.shields.io/github/commit-activity/m/3648TJSpartans/SkeletonRepository)](https://github.com/3648TJSpartans/SkeletonRepository/pulse)
![Repo Size](https://img.shields.io/github/repo-size/3648TJSpartans/SkeletonRepository)
[![Contributors](https://img.shields.io/github/contributors/3648TJSpartans/SkeletonRepository)](https://github.com/3648TJSpartans/SkeletonRepository/graphs/contributors)
[![Open PRs](https://img.shields.io/github/issues-pr/3648TJSpartans/SkeletonRepository)](https://github.com/3648TJSpartans/SkeletonRepository/pulls)
[![License](https://img.shields.io/github/license/3648TJSpartans/SkeletonRepository)](https://github.com/3648TJSpartans/SkeletonRepository/blob/main/LICENSE)

This is our team's template repository for future First Robotics Competitions. It contains most of the basics for our swerve drive bots, including AprilTag localization, Swerve, LEDs, Vision, and networking.

## Prerequisites
Java 25 or later
Github [CLI](https://cli.github.com/) or [Desktop](https://github.com/apps/desktop)
**OR**
[git](https://git-scm.com/) signed into your github account with access to the repository. Recommended sign in via the [GitHub CLI](https://cli.github.com/)


## Setup
We recommend using [WPILib 2027 flavored VSCode](https://github.com/wpilibsuite/allwpilib/releases) but if you like you can use another editor like [Zed](https://zed.dev/) with independant plugins and your own java installation. WPILib's vscode installer is the easiest to setup and use. Installing [WPILib 2027](https://github.com/wpilibsuite/allwpilib/releases) automatically sets up Java 25 and all other dependancies for this repo, it is highlighy recommended that in vscode you install the [Java extension pack](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack)

## Building

**Java 25+ Must Be Used to Build**

All you need to do is run ./gradlew build, if this fails <u>*double check*</u>  your active java version and make sure your local copy of the repository is up-to-date (git pull).

## Getting your changes into the [main branch](https://github.com/3648TJSpartans/SkeletonRepository/tree/main)
1. Publish a new branch with your changes (google how to do this in vscode if you don't know how)
2. Open a pull request (on the github website) for another code team member to review
3. Watch the magic happen

## Style guide
Unless otherwise specified, this repository should follow [Google's Java style guide](https://google.github.io/styleguide/javaguide.html). Formatting will be automatically enforced by Spotless, but you are expected to use correct naming conventions and the like.

## [Shortcut](https://app.shortcut.com/)
If you do not know how to use [Shortcut](https://app.shortcut.com/) feel free to ask any other member of the code team.
Shortcut is used as a way to implement [agile](https://en.wikipedia.org/wiki/Agile_software_development) into our development workflow. 

