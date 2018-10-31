# unique task id
uint32 id

# the state of the task. Valid values: 'initialized', 'running', 'waiting for package', ...
string status

# timings (0 if not yet set)

# time when the associated request was created
time requestCreateTime

# timestamp when the task was started
time runTime

# timestamp when the load was acknowledged
time loadTime

# timestamp when the unload was acknowledged
time unloadTime

# estimated duration from run to unload estimated by the roadmap planner
duration estimatedDuration

# task data

# id of the assigned robot
string robot

# Package this task is dealing with.
Package package

# The source and the target of this task
uint32 sourceTray
uint32 targetTray

