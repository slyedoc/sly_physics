My physics system needs a lot more work, biggest issues where:
 - New Type pattern is kind of a pain 
 - Need a transform gizmo for editing
 - Spent 2 days trying to make a physics based character controller and had to many issues, this lead to me not using the physics much
 - Really need to add mesh to convex collider back
 - Transform Hierarchy, I don't support it thats a huge problem
 - Need sensors


BASE

stack_cube/100          time:   [1.2092 s 1.2224 s 1.235 s]
                        thrpt:  [80.940  elem/s 81.806  elem/s 82.698  elem/s]
                 change:
                        time:   [-2.0115% -0.5492% +0.9152%] (p = 0.50 > 0.05)
                        thrpt:  [-0.9069% +0.5522% +2.0528%]
                        No change in performance detected.
stack_cube/500          time:   [4.2547 s 4.2815 s 4.3078 s]                            
                        thrpt:  [116.07  elem/s 116.78  elem/s 117.52  elem/s]

stack_sphere/100        time:   [888.47 ms 897.41 ms 906.60 ms]                           
                        thrpt:  [110.30  elem/s 111.43  elem/s 112.55  elem/s]
stack_sphere/500        time:   [2.7063 s 2.7215 s 2.7372 s]                              
                        thrpt:  [182.67  elem/s 183.72  elem/s 184.75  elem/s]
