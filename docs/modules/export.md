# `export.py` — CLI entry point

Backs the `onshape-to-robot` console script. Thin orchestrator over
`Config`, `RobotBuilder`, processors, and exporters.

## `main()`

1. Calls `load_dotenv(find_dotenv(usecwd=True))` — `.env` files can live
   anywhere up the tree from CWD.
2. Parses arguments:

   | Flag              | Effect |
   |-------------------|--------|
   | `robot_path`      | Positional, path to the robot directory. |
   | `--retrieve`      | Retrieval only — produces `robot.pkl`. Skips processors and export. |
   | `--save-pickle`   | Retrieve + save `robot.pkl`, then proceed with processors + export. |
   | `--convert`       | Skip retrieval; load `robot.pkl`, run processors + export. |
   | `--safe`          | Force the default processor registry (minus unsafe ones) and skip `post_import_commands`. |
   | `--version`       | Prints `onshape-to-robot {version}`. |

3. Builds `Config(robot_path, safe=args.safe)`.
4. Constructs the exporter early so the config is validated before doing
   any network work:

   ```python
   "urdf"   → ExporterURDF(config)
   "sdf"    → ExporterSDF(config)
   "mujoco" → ExporterMuJoCo(config)
   ```

   Unknown format → raises.
5. If not `--convert`: instantiates `RobotBuilder(config)` (retrieval).
   `robot = robot_builder.robot`.
6. If `--retrieve` or `--save-pickle`: dumps `robot` to
   `{output_directory}/robot.pkl`.
7. If `--convert`: loads `robot` from `{output_directory}/robot.pkl`.
8. If not `--retrieve`:
   - Runs `for processor in config.processors: processor.process(robot)`.
   - Calls `exporter.write_xml(robot, "{output_directory}/{output_filename}.{ext}")`.
   - Executes `config.post_import_commands` via `os.system` unless
     `--safe` is set.
9. `finally:` calls `robot_builder.close()` to free GLB temp dirs, even if
   something raised downstream.

Errors are caught only to print a red `ERROR: {e}` via `message.error`
before re-raising.
