import pandas as pd
import os

def calculate_torque(speeds, horsepowers):
    torques = []
    for speed, hp in zip(speeds, horsepowers):
        torque = (9550 * hp) / speed
        torques.append(torque)
    return torques


# Load the two Excel files
torque_file = "OpenLAP_matlab/vehicle_descriptions/Rhode Rage.xlsx"
intake_file = "sweep.xlsx"

df_torque = pd.read_excel(torque_file, sheet_name=None)  # Load all sheets
df_intake = pd.read_excel(intake_file)

# Get unique intake runner lengths
intake_lengths = df_intake['INTAKE_RUNNER_LENGTH'].unique()
print(intake_lengths)

# Create a folder to save the output files if it doesn't exist
output_folder = "Output_Files"
os.makedirs(output_folder, exist_ok=True)

# Loop over each intake runner length and create a new xlsx file for each one
for i in range(len(intake_lengths)):
    # Retrieve the data for the specific intake runner length
    filtered_data = df_intake[df_intake['INTAKE_RUNNER_LENGTH'] == intake_lengths[i]]
    speeds = filtered_data['SPEED'].values
    torques = filtered_data['HORSEPOWER'].values

    # Create a copy of the Rhode Rage excel file and edit the Torque Curve sheet
    new_torque_file = os.path.join(output_folder, f"Rhode_Rage_{i}.xlsx")
    
    # Copy the original data
    with pd.ExcelWriter(new_torque_file, engine='xlsxwriter') as writer:
        # Write existing sheets
        for sheet_name, sheet_data in df_torque.items():
            sheet_data.to_excel(writer, sheet_name=sheet_name, index=False)
        
        # Now add/update the Torque Curve sheet with the new data
        torque_curve_data = pd.DataFrame({
            'Engine Speed [rpm]': speeds,
            'Torque [Nm, adjusted 0.7/0.9]' : torques,#calculate_torque(speeds, torques),
            'Torque [Nm]': torques#calculate_torque(speeds, torques)
        })
        
        torque_curve_data.to_excel(writer, sheet_name='Torque Curve', index=False)
