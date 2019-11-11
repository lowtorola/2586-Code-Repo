package frc.robot;

public class PID {
	// Made private because of getValue() functions
	private double kP = .0275;
	private double kI = 0;
	private double kD = 0;
	private double target = 0;
	private double position = 0;
	private double kDeadband = 0;
	
	// Always private
	private double currentError = 0;
	private double lastError = 0;
	private double aI = 0;
	private double P_Output, I_Output, D_Output;
	
	public PID(double newP, double newI, double newD, double newDeadband) {
		setP(newP);
		setI(newI);
		setD(newD);
		setDeadband(newDeadband);
	}
	
	public PID(double newP, double newI) {
		setP(newP);
		setI(newI);
	}
	
	private void setP(double newP) {
		if(newP < 0)
		{
			throw new IllegalArgumentException("Illegal kP Value: " + newP + "\nValue cannot be negative.");
		}
		else {
			kP = newP;
		}
	}
	
	private void setI(double newI) {
		if(newI < 0)
		{
			throw new IllegalArgumentException("Illegal kI Value: " + newI + "\nValue cannot be negative.");
		}
		else {
			kI = newI;
		}
	}
	
	private void setD(double newD) {
		if(newD < 0)
		{
			throw new IllegalArgumentException("Illegal kD Value: " + newD + "\nValue cannot be negative.");
		}
		else {
			kD = newD;
		}
	}

	private void setDeadband(double newDeadband) {
		if(newDeadband < 0)
		{
			throw new IllegalArgumentException("Illegal kDeadband Value: " + newDeadband + "\nValue cannot be negative.");
		}
		else {
			kDeadband = newDeadband;
		}
	}
	
	void setPID(double newP, double newI, double newD) {
		setP(newP);
		setI(newI);
		setD(newD);
	}
	
	void setPID(double newP, double newI) {
		setP(newP);
		setI(newI);
		setD(0);
	}
	
	private void calcError(){
		currentError = target - position;
	}
	
	private void calcP_Output() {
		calcError();
		P_Output = kP * currentError;
	}
	
	private void calcI_Output() {
		calcError();
		I_Output = (kI * currentError) + aI;
		calcAccumulatedI();
	}
	
	private void calcAccumulatedI() {
		calcError();
		if(currentError != lastError)
		{
			aI += kI * currentError;
		}
	}
	
	private void calcD_Output() {
		calcError();
		if(currentError != lastError) {
			D_Output = kD * (currentError - lastError);
		}
		updateError();
	}
	
	private double getP_Output() {
		calcP_Output();
		return P_Output;
	}
	
	private double getI_Output() {
		calcI_Output();
		return I_Output;
	}
	
	/*private */double getD_Output() {
		calcD_Output();
		return D_Output;
	}
	
	double getTarget() {
		return target;
	}
	
	double getPosition() {
		return position;
	}
	
	double getP() {
		return kP;
	}
	
	double getI() {
		return kI;
	}
	
	double getD() {
		return kD;
	}
	
	double getCurrentError() {
		calcError();
		return currentError;
	}
	
	double getLastError() {
		updateError();
		return lastError;
	}
	
	double getAccumulatedI() {
		calcAccumulatedI();
		return aI;
	}
	
	double getOutput(double pos, double tar) {
		target = tar;
		position = pos;
		double output = getP_Output() + getI_Output() + getD_Output();
		if (Math.abs(output) < kDeadband) {
			return 0;
		} else {
		return output;
		}
	}
	
	private void updateError() {
		calcError();
		lastError = currentError;
	}
	
	void resetPID() {
		kP = 0;
		kI = 0;
		kD = 0;
	}
	
	void resetError() {
		aI = 0;
		lastError = 0;
	}
	
}