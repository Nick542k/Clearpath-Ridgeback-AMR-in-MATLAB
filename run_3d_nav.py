import matlab.engine

def main():
    print("Connecting to active MATLAB session...")
    eng = matlab.engine.connect_matlab('MATLAB_2132')
    
    print("Opening 'ridgeback_3d_nav' in Simulink GUI...")
    eng.open_system('ridgeback_3d_nav', nargout=0)
    
    print("Running 3D Simscape Multibody simulation (Mechanics Explorer will animate)...")
    eng.eval("out = sim('ridgeback_3d_nav');", nargout=0)
    
    print("3D simulation finished successfully!")
    tout = eng.eval("out.tout")
    print(f"Simulation simulated {len(tout)} time steps across 3D environment!")

if __name__ == '__main__':
    main()
