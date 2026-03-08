double RAJ(double distanceFromTarget, double kP)
{
  return distanceFromTarget*kP;
}

double RAJ(double target, double currentPos, double kPDivisor) 
{
  double kP = 1 / (target*kPDivisor); 
  return RAJ(target - currentPos,kP);
}

double motorRAJ(double target, double currentPos, double Velocity, double decelerateDistance, double acceleration)
{
  return ((RAJ(target-currentPos,1/decelerateDistance)*acceleration) + (Velocity * (1 - acceleration)))/2;
}

double clamp(double value, double min, double max)
{
  if(min>value)
  {
    return min;
  }
  else if(max < value)
  {
    return max;
  }
  return value;
}