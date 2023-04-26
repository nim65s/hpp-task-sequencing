# Example script to run in tiago deburring demo.

from hpp.corbaserver import Client, loadServerPlugin
from hpp.corbaserver.task_sequencing import Client as SolverClient

loadServerPlugin("corbaserver", "task-sequencing.so")

s = SolverClient()
n=2
s.solver.create(n)
s.solver.setErrorThreshold(1e-6)
s.solver.setMaxIterations(30)

for i in range(n):
    s.solver.addConstraint('tiago/gripper grasps driller/handle', i)
    s.solver.addConstraint('part/root_joint', i)
    if i != 0:
        s.solver.addEqualityConstraint('tiago/root_joint', i, 0)

q0[:4] = [0, 1, 0, -1]
e1 = 'driller/drill_tip > part/handle_4 | 0-0_01'
res, q1, err = graph.generateTargetConfig(e1, q0, q0)
assert(res)
e2 = 'driller/drill_tip > part/handle_6 | 0-0_01'
q0[:4] = [-0.1, 1, 0, -1]
res, q2, err = graph.generateTargetConfig(e2, q0, q0)
assert(res)

s.solver.addConstraint('driller/drill_tip pregrasps part/handle_4', 0)
s.solver.addConstraint('driller/drill_tip pregrasps part/handle_6', 1)
q = q1 + q2

s.solver.setRightHandSideFromVector(q)
res, q3, err = s.solver.solve(q)
