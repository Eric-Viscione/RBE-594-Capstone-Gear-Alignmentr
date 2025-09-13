import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime, timedelta

# Project data, structured by phase and task with new durations and start times.
# The total duration has been condensed to 11 weeks to meet the December 12th deadline.
data = [
    {
        "phase": "Phase 1: Simulation Setup & Basic Rigging (2 weeks)",
        "tasks": [
            {"name": "1.1. Select and set up a robotics simulator.", "start_week": 0, "duration": 1},
            {"name": "1.2. Create a virtual environment...", "start_week": 1, "duration": 1},
            {"name": "1.3. Design and model gears...", "start_week": 1, "duration": 1},
            {"name": "1.4. Integrate two robot models...", "start_week": 1.5, "duration": 0.5},
            {"name": "1.5. Attach the gripper and gear end effectors...", "start_week": 1.5, "duration": 0.5},
        ],
    },
    {
        "phase": "Phase 2: Sensor & Perception Integration (3 weeks)",
        "tasks": [
            {"name": "2.1. Integrate a simulated camera...", "start_week": 2, "duration": 1},
            {"name": "2.2. Implement simulated sensors...", "start_week": 2.5, "duration": 1.5},
            {"name": "2.3. Develop a perception system...", "start_week": 3, "duration": 2},
        ],
    },
    {
        "phase": "Phase 3: Control Algorithm Development (6 weeks)",
        "tasks": [
            {"name": "3.1. Develop the 'Picker Robot's' control algorithm...", "start_week": 5, "duration": 3.5},
            {"name": "3.2. Develop the 'Starter Robot's' control algorithm...", "start_week": 6, "duration": 3.5},
            {"name": "3.3. Integrate the algorithms...", "start_week": 8.5, "duration": 2},
            {"name": "3.4. Implement a motion planning algorithm...", "start_week": 9.5, "duration": 1.5},
        ],
    },
    {
        "phase": "Phase 4: Testing & Validation (2 weeks)",
        "tasks": [
            {"name": "4.1. Develop a testing framework...", "start_week": 9.5, "duration": 1},
            {"name": "4.2. Collect and analyze performance metrics...", "start_week": 10.5, "duration": 1},
            {"name": "4.3. Refine the control algorithms...", "start_week": 10.5, "duration": 0.5},
        ],
    },
    {
        "phase": "Phase 5: Documentation & Presentation (2 weeks)",
        "tasks": [
            {"name": "5.1. Draft the comprehensive written report...", "start_week": 10.5, "duration": 1.5},
            {"name": "5.2. Prepare a public presentation...", "start_week": 11.5, "duration": 0.5},
        ],
    },
]

# Set a start date for the project (e.g., Nov 1, 2025) to meet the Dec 12, 2025 deadline.
start_date = datetime(2025, 9, 15)

# Prepare data for plotting
tasks = []
start_dates = []
durations = []
colors = []
color_map = plt.cm.get_cmap('viridis', len(data))

# Iterate through each phase and its tasks
for i, phase in enumerate(data):
    for task in phase["tasks"]:
        tasks.append(task["name"])
        start_dates.append(start_date + timedelta(weeks=task["start_week"]))
        durations.append(timedelta(weeks=task["duration"]))
        colors.append(color_map(i))

# Create the plot
fig, ax = plt.subplots(figsize=(12, 8))

# Create the bars for the Gantt chart
for i, (task_name, start_date, duration, color) in enumerate(zip(tasks, start_dates, durations, colors)):
    ax.barh(task_name, duration, left=start_date, color=color, height=0.6)
    # Add a label to the bar with its duration, formatted for less than one week
    duration_in_weeks = duration.total_seconds() / (7 * 24 * 3600)
    if duration_in_weeks < 1:
        label_text = f' {duration_in_weeks:.2f} weeks'
    else:
        label_text = f' {int(duration_in_weeks)} weeks'
    ax.text(start_date + duration, i, label_text, va='center', ha='left')

# Set labels, title, and grid
ax.set_xlabel('Timeline (Weeks)')
ax.set_title('Project Timeline: SIRGAS')
ax.grid(True, axis='x', linestyle='--', alpha=0.6)

# Format the x-axis to show dates in a readable format
ax.xaxis.set_major_formatter(mdates.DateFormatter('%b %d'))
ax.xaxis.set_major_locator(mdates.WeekdayLocator(byweekday=mdates.MO))
fig.autofmt_xdate()

# Invert the y-axis to show the first task at the top
ax.invert_yaxis()

plt.tight_layout()
plt.show()
