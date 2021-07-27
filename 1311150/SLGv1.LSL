 ____________________________________________ 
*Solution commenced on 23/07/2021 at 17:03:35 
  
 Strand7 [2.4.6][Solver Build: 24141222] (32-Bit) 
  
 ANALYSIS TYPE               : LINEAR STATIC 
  
 COMPUTER NAME               : JB-NOTEBOOK 
 USER LOGON NAME             : John B 
 CPU                         : Intel(R) Core(TM) i7-8750H CPU @ 2.20GHz 
 USABLE PHYSICAL MEMORY      : 4.0 GB 
 USABLE VIRTUAL MEMORY       : 3.0 GB 
  
 MODEL FILE                  : "C:\Users\John B\Projects_Git\BRP\2021\2B\1311150\SLGv1.st7" 
 RESULT FILE                 : "C:\Users\John B\Projects_Git\BRP\2021\2B\1311150\SLGv1.LSA" 
 SCRATCH PATH                : "C:\Users\John B\Strand7\Tmp\" 
  
 TOTALS 
 Nodes                       : 455 
 Beams                       : 454 
 Plates                      :   0 
 Bricks                      :   0 
 Links                       :   0 
  
 SOLVER UNITS 
 Length                      : in 
 Mass                        : lbm 
 Force                       : lbf 
 Stress                      : psi 
  
 FREEDOM CASE                : "Freedom Case 1" 
  
 LOAD CASES                  : "Load Case 1" 
  
 STORAGE SCHEME              : Sparse  
 SORTING METHOD              : AMD  
 SOLUTION TYPE               : Direct  
  
 NUMBER OF EQUATIONS         : 1360 
 MATRIX FILL-IN              : 0.0% 
 [K] MATRIX SIZE             : 53.0 KB 
 OPTIMUM RAM NEEDED          : 64.0 KB 
 FREE SCRATCH SPACE          : 631.2 GB 
  
*WARNING[237]:Default freedom conditions appear to be over-constrained - one or more elements may require additional freedom. 
  
 SUMMATION OF APPLIED LOADS (Name: "Load Case 1") 
              FX           FY           FZ           MX           MY           MZ      
 Beams    0.00000E+00 -5.57319E+05  0.00000E+00  0.00000E+00  0.00000E+00 -3.53654E+03 
 Total    0.00000E+00 -5.57319E+05  0.00000E+00  0.00000E+00  0.00000E+00 -3.53654E+03 
 Vector   0.00000E+00 -5.53421E+05  0.00000E+00  0.00000E+00  0.00000E+00 -3.53654E+03 
  
 SUMMATION OF MOMENTS OF APPLIED LOADS ABOUT THE ORIGIN [Load Vector] 
              MXo          MYo          MZo           
          0.00000E+00  0.00000E+00 -1.49424E+09 
  
 Reducing 1360 Equations (Using 64.0 KB RAM)... 
  
 MAXIMUM PIVOT               : 7.213303E+12 (Node 312 RZ) 
 MINIMUM PIVOT               : 3.181121E+06 (Node 2 DX) 
  
 Results for 1 Load Case... 
  
 MAXIMUM DISPLACEMENT MAGNITUDES 
   Case        DX           DY           DZ           RX           RY           RZ       Name 
     1     0.00000E+00  1.30204E+00  0.00000E+00  0.00000E+00  0.00000E+00  3.07929E-03  "Load Case 1" 
  
 DIRECT SUMMATION OF NODE REACTION FORCES 
   Case        FX           FY           FZ           MX           MY           MZ       Name 
     1     0.00000E+00  5.57319E+05  0.00000E+00  0.00000E+00  0.00000E+00  1.76982E-03  "Load Case 1" 
  
 TOTAL CPU TIME              : 0.359 Seconds 
  
*Solution completed on 23/07/2021 at 17:03:35 
*Solution time: 1 Second 
  
 
*SUMMARY OF MESSAGES
*Number of Notes    : 0
*Number of Warnings : 1
*Number of Errors   : 0
 
