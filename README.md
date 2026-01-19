<h2 align="center"><em><code>Robot Tour</code></em></h2>

C++ code for the Robot Tour Science Olympiad event, running on the Polulu Romi 32U4.

## Features
- Encoder-based positional turning
- Time-scaled linear motion
- PI motor control
- String-based command parsing

## Movement Encoding

```bash
char moves[200] = "F30 R F L F35 L L F R";
```

- Each spaced token represents one individual command
- Commands are single-character instructions with optional numeric inputs

## Supported Commands

- `F[#]` - Forward (default 50 units or specified distance)
- `B[#]` - Backward (default 50 units or specified distance)
- `L` - Left turn
- `R` - Right turn
- `S` - Drive predefined start distance
- `E` - Drive predefined end distance

## Parsing & Planning

### Tokenization
- The `moves` string is tokenized using `strtok`
- Tokens are stored as pointers to avoid copying memory
- A pre-pass analyzes the entire path before execution

```bash
ptr = strtok(moves, " ");
```

## Pre-Execution Analysis

Before any motion occurs, the program:
- Counts the total number of turns
- Computes total linear distance traveled
- Determines how to allocate time proportionally across movements

## Time-Scaled Motion Model

```bash
double totalDriveTime = targetTime - totalTurnTime - correctionFactor * totalDistance;
```

- A global targetTime defines total runtime
- Turns are assigned a fixed duration
- Drive time is distributed proportionally based on distance
- Helps ensure consistent overall execution timing across different paths

## Motion Execution
- Turns
  - Encoder-based position control
  - Uses tuned encoder counts for ~90° rotations
- Linear Motion
  - Time-based driving scaled by distance contribution
- Drift Compensation
  - Small corrective turns applied after forward motion
 
```bash
chassis.driveWithTime(dist, dist / totalDist * totalDriveTime);
```
