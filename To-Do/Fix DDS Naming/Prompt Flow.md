Used this prompt; Please review these two documents, starting here; 
  '/home/finn/Documents/ardu_ws/Claude Documentation/Pattern formation 
  methodology/Master-Multi-Drone-3D-Formation-Control-Plan.md' then here 
  '/home/finn/Documents/ardu_ws/Claude Documentation/Pattern formation 
  methodology/Phase-2-Formation-Control-Testing-Plan.md'

  In order to have the agent gain an understanding of the focused objective of phase 2. And then the capabilities / conventions of ardupilot DDS.  
  Then I used the following prompt to have said agent review each formation control file, looking for naming convention errors, and writing a plan to fix the errors it finds.

`Please review the following file; '/home/finn/Documents/ardu_ws/src/formation_control
  /formation_control/drone_interface.py' and ultra think while looking for any errors 
  in the code that do not agree with '/home/finn/Documents/ardu_ws/Claude 
  Documentation/Pattern formation 
  methodology/Phase-2-Formation-Control-Testing-Plan.md'`

  Next I used a second agent, and brought it up to speed using the same initial prompt. But then asked it to use the plan / instructions written by the first agent to exacute the fixes that needed to happen;

  Thank you, next I'd like you to impliment the following instructions; 
  '/home/finn/Documents/ardu_ws/src/formation_control/To-Do/swarm-takeoff-commander-dds
  -migration-analysis.md' on this file; '/home/finn/Documents/ardu_ws/src/formation_con
  trol/formation_control/swarm_takeoff_commander.py'