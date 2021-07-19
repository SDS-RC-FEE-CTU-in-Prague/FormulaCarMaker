Name:Traffic.Sheer.Vel
Unit:m/s
Description:-
SetCompilerSwitch("EnableRangeCheck",1)
//FormulaCalculator: Assign A
A=DS("Traffic.Sheer.LongVel")
//FormulaCalculator: Assign B
B=DS("Traffic.Sheer.LatVel")
//FormulaCalculator: Assign C
C=0
//FormulaCalculator: Assign D
D=0
//FormulaCalculator: Calculation:
result=sqrt(Pow(A,2)+Pow(B,2))
return result