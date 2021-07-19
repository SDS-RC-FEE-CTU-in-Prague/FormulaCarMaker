//FormulaCalculator generated formula
Name:Traffic.Ahead.Vel
Unit:m/s
Description:-
SetCompilerSwitch("EnableRangeCheck",1)
//FormulaCalculator: Assign A
A=DS("Traffic.Ahead.LongVel")
//FormulaCalculator: Assign B
B=DS("Traffic.Ahead.LatVel")
//FormulaCalculator: Assign C
C=0
//FormulaCalculator: Assign D
D=0
//FormulaCalculator: Calculation:
result=sqrt(Pow(A,2)+Pow(B,2))
return result
