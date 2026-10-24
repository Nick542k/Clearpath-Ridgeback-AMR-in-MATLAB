"""
simulink_connector.py
Connect and interact with MATLAB Simulink from Python using the MATLAB Engine API.
"""

import matlab.engine
from typing import Optional, Any, Dict


class SimulinkBridge:
    def __init__(self, shared_session_name: Optional[str] = None):
        """
        Initialize the connection to MATLAB.
        
        :param shared_session_name: If provided, connects to a shared MATLAB session.
                                    If None and shared sessions exist, connects to the first one.
                                    Otherwise, launches a new MATLAB instance.
        """
        sessions = matlab.engine.find_matlab()
        if shared_session_name:
            print(f"Connecting to shared MATLAB session '{shared_session_name}'...")
            self.eng = matlab.engine.connect_matlab(shared_session_name)
        elif len(sessions) > 0:
            print(f"Connecting to existing shared MATLAB session '{sessions[0]}'...")
            self.eng = matlab.engine.connect_matlab(sessions[0])
        else:
            print("Starting a new MATLAB engine instance (this may take a few seconds)...")
            self.eng = matlab.engine.start_matlab()

        print("Connected to MATLAB successfully!")

    def open_simulink_library(self):
        """Open the Simulink Start Page / Library browser."""
        self.eng.simulink(nargout=0)

    def load_model(self, model_name: str):
        """
        Load a Simulink model (.slx) into memory without opening its GUI window.
        Use open_model() if you want the visual editor to appear.
        """
        print(f"Loading Simulink model '{model_name}'...")
        self.eng.load_system(model_name, nargout=0)

    def open_model(self, model_name: str):
        """Open the Simulink model inside the MATLAB graphical editor."""
        print(f"Opening Simulink model '{model_name}' in editor...")
        self.eng.open_system(model_name, nargout=0)

    def set_workspace_variable(self, var_name: str, value: Any):
        """Set a variable in the MATLAB base workspace for the model to use."""
        self.eng.workspace[var_name] = value

    def get_workspace_variable(self, var_name: str) -> Any:
        """Retrieve a variable from the MATLAB base workspace."""
        return self.eng.workspace[var_name]

    def set_block_parameter(self, block_path: str, parameter: str, value: str):
        """
        Set a parameter of a specific block in the model.
        Example: set_block_parameter('my_model/Gain_1', 'Gain', '10.5')
        """
        self.eng.set_param(block_path, parameter, str(value), nargout=0)

    def run_simulation(self, model_name: str, stop_time: Optional[float] = None) -> Any:
        """
        Run the Simulink simulation and return the Simulink.SimulationOutput object.
        """
        print(f"Running simulation for '{model_name}'...")
        if stop_time is not None:
            sim_cmd = f"sim('{model_name}', 'StopTime', '{stop_time}')"
        else:
            sim_cmd = f"sim('{model_name}')"
        
        # Run simulation and store result in MATLAB workspace
        self.eng.eval(f"simOut = {sim_cmd};", nargout=0)
        return self.eng.workspace['simOut']

    def close_model(self, model_name: str, save: bool = False):
        """Close the model."""
        save_flag = 1 if save else 0
        self.eng.close_system(model_name, save_flag, nargout=0)
        print(f"Closed model '{model_name}' (Saved: {save}).")

    def disconnect(self):
        """Disconnect and terminate the MATLAB engine session."""
        print("Closing MATLAB engine session...")
        self.eng.quit()
        print("Disconnected.")


if __name__ == "__main__":
    bridge = SimulinkBridge()
    try:
        # Example 1: Verify Simulink is installed and license is accessible
        is_simulink_installed = bridge.eng.eval("exist('simulink', 'file') == 2")
        print(f"Simulink availability check: {'Available' if is_simulink_installed else 'Not Found'}")

        # Example 2: Launch or load your model
        # bridge.load_model("my_model")
        # bridge.set_workspace_variable("Kp", 1.5)
        # sim_out = bridge.run_simulation("my_model", stop_time=10.0)
        # bridge.close_model("my_model")

    finally:
        bridge.disconnect()
