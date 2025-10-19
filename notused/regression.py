import pandas as pd
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from scipy.optimize import minimize_scalar

# === STEP 1: Load and clean data ===
file_path = "measurements_rotors.csv"  # Update path if needed

df = pd.read_csv(file_path)

# Extract voltage labels from row 1 (second row in 0-based indexing)
voltage_labels = df.iloc[1, 1:].astype(float).values

# Extract PWM values from column 0, starting from row 2
pwm_values = df.iloc[2:, 0].astype(float).values

# Extract thrust values from row 2 onwards, columns 1 onward
thrust_values = df.iloc[2:, 1:].astype(float).values

# Construct a clean long-form dataset (PWM, Voltage, Thrust)
data = []
for col_idx, voltage in enumerate(voltage_labels):
    for row_idx, pwm in enumerate(pwm_values):
        thrust = thrust_values[row_idx, col_idx]
        if not np.isnan(thrust):
            data.append((pwm, voltage, thrust))

clean_df = pd.DataFrame(data, columns=["PWM", "Voltage", "Thrust"])

# === STEP 2: Fit polynomial regression model ===
X = clean_df[["PWM", "Voltage"]].values
y = clean_df["Thrust"].values

model = make_pipeline(PolynomialFeatures(degree=2, include_bias=False), LinearRegression())
model.fit(X, y)

# === STEP 3: Invert the model to get PWM from (Thrust, Voltage) ===
def get_pwm(desired_thrust, voltage, pwm_bounds=(10, 255)):
    """
    Returns the PWM value that produces the desired thrust at a given voltage.
    """
    def objective(pwm):
        input_features = np.array([[pwm, voltage]])
        predicted_thrust = model.predict(input_features)[0]
        return abs(predicted_thrust - desired_thrust)
    
    res = minimize_scalar(objective, bounds=pwm_bounds, method='bounded')
    if res.success:
        return res.x
    else:
        raise RuntimeError("PWM could not be found.")

# === STEP 4: Example usage ===
if __name__ == "__main__":
    desired_thrust = 12  # grams
    voltage = 3.7        # volts
    pwm = get_pwm(desired_thrust, voltage)
    print(f"Required PWM for {desired_thrust} g thrust at {voltage} V: {pwm:.2f}")
