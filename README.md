# Cloning

Clone the repo to your working directory:

```
git clone https://github.com/TripleHelixProgramming/ChargedUp.git
```

# Building

```
./gradlew build
```

# Running in Simulation

To open the simulation ui:

```
./gradlew simulateNative`
```

You can use the WASD to control the robot in Teleop mode, and Autonomous mode will run a basic trajectory.

# C++ Intellisense in Visual Studio Code

Install the clangd extension in VS Code, and disable the default C++ extension.

Run this to generate the `compile_commands.json` file:

```
./gradlew generateCompileCommands
```

Then add a `.vscode/settings.json` to your cloned repo with the following contents:

For macOS:
```json
{
    "clangd.arguments": [
        "-compile-commands-dir=build/compile_commands/osxuniversal"
    ]
}
```

You may need to reload the window for the Intellisense to update.

# Code formatting

Autoformatting can be performed using wpiformat. Make sure clang-format is installed first, then run this at the root of the repo:

```
python3 -m pip install wpiformat
python3 -m wpiformat
```
