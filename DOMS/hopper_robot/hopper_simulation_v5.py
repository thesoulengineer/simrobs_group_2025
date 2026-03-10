import mujoco
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution
import random
import time

class COTOptimizer:
    def __init__(self, xml_path="DOMS/tutorials/two_link_hopper_2_dof.xml", sim_duration=3.0):
        self.xml_path = xml_path
        self.sim_duration = sim_duration
        
        # Load model once to get information
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.total_mass = np.sum(self.model.body_mass)
        
        # Joint indices (same as in simulator)
        self.x_qadr = self.model.jnt_qposadr[self.model.joint("x_slide").id]
        self.z_qadr = self.model.jnt_qposadr[self.model.joint("z_slide").id]
        self.hip_qadr = self.model.jnt_qposadr[self.model.joint("hip").id]
        self.knee_qadr = self.model.jnt_qposadr[self.model.joint("knee").id]
        
        self.hip_vadr = self.model.jnt_dofadr[self.model.joint("hip").id]
        self.knee_vadr = self.model.jnt_dofadr[self.model.joint("knee").id]
        
        # Bounds for optimization parameters
        # [hip_amp, hip_offset, hip_phase, knee_amp, knee_offset, knee_phase, frequency, hip_kp, hip_kd, knee_kp, knee_kd]
        self.param_bounds = [
            (10, 60),        # hip_amp (degrees)
            (-30, 30),       # hip_offset (degrees)
            (0, 2*np.pi),    # hip_phase (radians)
            (10, 80),        # knee_amp (degrees)
            (-30, 30),       # knee_offset (degrees)
            (0, 2*np.pi),    # knee_phase (radians)
            (1.0, 4.0),      # frequency (Hz)
            (50, 300),       # hip_kp
            (5, 50),         # hip_kd
            (50, 300),       # knee_kp
            (5, 50)          # knee_kd
        ]
        
        # Best solution tracking
        self.best_cot = float('inf')
        self.best_params = None
        self.best_trajectory = None
        
        # History for plotting
        self.generation_history = []
        self.best_cot_history = []
        self.avg_cot_history = []
        
    def simulate_with_params(self, params, render=False):
        """Run a simulation with given parameters and return COT"""
        # Unpack parameters
        (hip_amp, hip_offset, hip_phase, 
         knee_amp, knee_offset, knee_phase,
         frequency, hip_kp, hip_kd, knee_kp, knee_kd) = params
        
        # Create new data instance for this simulation
        data = mujoco.MjData(self.model)
        
        # Set initial crouched position
        data.qpos[self.hip_qadr] = np.radians(-30)
        data.qpos[self.knee_qadr] = np.radians(50)
        mujoco.mj_forward(self.model, data)
        
        # Tracking variables
        x_history = [data.qpos[self.x_qadr]]
        work_history = [0.0]
        cumulative_work = 0.0
        last_energy = self._compute_total_energy(self.model, data)
        
        # Create viewer if rendering
        viewer = None
        if render:
            import mujoco_viewer
            viewer = mujoco_viewer.MujocoViewer(self.model, data)
        
        step = 0
        max_steps = int(self.sim_duration / self.model.opt.timestep)
        
        try:
            for _ in range(max_steps):
                t = data.time
                
                # Compute desired trajectories
                hip_desired = hip_amp * np.sin(2 * np.pi * frequency * t + hip_phase) + hip_offset
                knee_desired = knee_amp * np.sin(2 * np.pi * frequency * t + knee_phase) + knee_offset
                
                # Convert to radians
                hip_desired_rad = np.radians(hip_desired)
                knee_desired_rad = np.radians(knee_desired)
                
                # Get current states
                hip_current = data.qpos[self.hip_qadr]
                knee_current = data.qpos[self.knee_qadr]
                hip_vel = data.qvel[self.hip_vadr]
                knee_vel = data.qvel[self.knee_vadr]
                
                # PD control
                hip_torque = hip_kp * (hip_desired_rad - hip_current) - hip_kd * hip_vel
                knee_torque = knee_kp * (knee_desired_rad - knee_current) - knee_kd * knee_vel
                
                # Apply control
                data.ctrl[0] = hip_torque
                data.ctrl[1] = knee_torque
                
                # Step simulation
                mujoco.mj_step(self.model, data)
                
                # Track mechanical work using energy method (more accurate)
                current_energy = self._compute_total_energy(self.model, data)
                energy_change = current_energy - last_energy
                if energy_change > 0:
                    cumulative_work += energy_change
                last_energy = current_energy
                
                # Log data
                if step % 10 == 0:
                    x_history.append(data.qpos[self.x_qadr])
                    work_history.append(cumulative_work)
                
                # Render if requested
                if render and viewer is not None:
                    viewer.render()
                    
                step += 1
                
        except Exception as e:
            print(f"Simulation error: {e}")
            return float('inf')
        finally:
            if render and viewer is not None:
                viewer.close()
        
        # Calculate COT
        if len(x_history) > 1:
            distance = x_history[-1] - x_history[0]
            total_work = work_history[-1]
            
            if distance > 0.01:  # Avoid division by zero and very small distances
                weight = self.total_mass * 9.81
                cot = total_work / (weight * distance)
                return cot
            else:
                return float('inf')  # Penalize no movement
        else:
            return float('inf')
    
    def _compute_total_energy(self, model, data):
        """Compute total mechanical energy of the system"""
        # Reset energy values
        data.energy[0] = 0  # Potential energy
        data.energy[1] = 0  # Kinetic energy
        
        # Compute potential energy
        mujoco.mj_energyPos(model, data)
        pe = data.energy[0]
        
        # Compute kinetic energy
        mujoco.mj_energyVel(model, data)
        ke = data.energy[1]
        
        return ke + pe
    
    def fitness_function(self, params):
        """Fitness function for genetic algorithm (minimize COT)"""
        cot = self.simulate_with_params(params)
        
        # Add penalties for invalid or unstable gaits
        if np.isinf(cot) or np.isnan(cot):
            return 1e6  # Large penalty
        
        # Penalize very high COT
        if cot > 10:
            return 10 + (cot - 10) * 0.1  # Gradual penalty
        
        return cot
    
    def optimize(self, population_size=50, generations=30, mutation_rate=0.1, crossover_rate=0.7):
        """Run genetic algorithm optimization"""
        
        print("=" * 60)
        print("OPTIMIZING JOINT TRAJECTORIES FOR MINIMUM COST OF TRANSPORT")
        print("=" * 60)
        print(f"Parameters to optimize: 11")
        print(f"Population size: {population_size}")
        print(f"Generations: {generations}")
        print(f"Simulation duration: {self.sim_duration}s")
        print(f"Total system mass: {self.total_mass:.3f} kg")
        print("=" * 60)
        
        # Initialize population
        population = []
        for _ in range(population_size):
            individual = []
            for bounds in self.param_bounds:
                individual.append(random.uniform(bounds[0], bounds[1]))
            population.append(individual)
        
        # Evolution loop
        for generation in range(generations):
            start_time = time.time()
            
            # Evaluate fitness for all individuals
            fitness_scores = []
            for i, individual in enumerate(population):
                fitness = self.fitness_function(individual)
                fitness_scores.append(fitness)
                
                # Track best solution
                if fitness < self.best_cot:
                    self.best_cot = fitness
                    self.best_params = individual.copy()
                    self.best_trajectory = self._get_trajectory_description(individual)
                    print(f"\n✨ New best COT: {self.best_cot:.4f} at generation {generation}")
            
            # Calculate statistics
            best_idx = np.argmin(fitness_scores)
            best_fitness = fitness_scores[best_idx]
            avg_fitness = np.mean(fitness_scores)
            
            self.generation_history.append(generation)
            self.best_cot_history.append(best_fitness)
            self.avg_cot_history.append(avg_fitness)
            
            # Print progress
            elapsed = time.time() - start_time
            print(f"Generation {generation:3d} | Best COT: {best_fitness:.4f} | "
                  f"Avg COT: {avg_fitness:.4f} | Time: {elapsed:.1f}s")
            
            # Select parents (tournament selection)
            parents = []
            for _ in range(population_size):
                tournament = random.sample(list(enumerate(fitness_scores)), 3)
                winner = min(tournament, key=lambda x: x[1])[0]
                parents.append(population[winner])
            
            # Create next generation
            next_population = []
            
            # Elitism: keep the best individual
            next_population.append(population[best_idx])
            
            while len(next_population) < population_size:
                # Select parents
                parent1 = random.choice(parents)
                parent2 = random.choice(parents)
                
                # Crossover
                if random.random() < crossover_rate:
                    child1, child2 = self._crossover(parent1, parent2)
                else:
                    child1, child2 = parent1.copy(), parent2.copy()
                
                # Mutation
                child1 = self._mutate(child1, mutation_rate)
                child2 = self._mutate(child2, mutation_rate)
                
                next_population.extend([child1, child2])
            
            # Trim if we overshot
            population = next_population[:population_size]
        
        print("\n" + "=" * 60)
        print("OPTIMIZATION COMPLETE")
        print("=" * 60)
        print(f"Best COT found: {self.best_cot:.4f}")
        print("\nBest Parameters:")
        self._print_params(self.best_params)
        
        return self.best_params, self.best_cot
    
    def _crossover(self, parent1, parent2):
        """Uniform crossover"""
        child1 = []
        child2 = []
        for i in range(len(parent1)):
            if random.random() < 0.5:
                child1.append(parent1[i])
                child2.append(parent2[i])
            else:
                child1.append(parent2[i])
                child2.append(parent1[i])
        return child1, child2
    
    def _mutate(self, individual, mutation_rate):
        """Gaussian mutation"""
        mutated = individual.copy()
        for i in range(len(mutated)):
            if random.random() < mutation_rate:
                # Add Gaussian noise (scale relative to parameter range)
                param_range = self.param_bounds[i][1] - self.param_bounds[i][0]
                noise = np.random.normal(0, 0.1 * param_range)
                mutated[i] += noise
                # Clip to bounds
                mutated[i] = max(self.param_bounds[i][0], 
                                min(self.param_bounds[i][1], mutated[i]))
        return mutated
    
    def _get_trajectory_description(self, params):
        """Get human-readable description of trajectories"""
        hip_amp, hip_offset, hip_phase, knee_amp, knee_offset, knee_phase, freq, _, _, _, _ = params
        
        desc = f"Hip: {hip_amp:.1f}° amplitude, {hip_offset:.1f}° offset, {hip_phase:.2f} rad phase\n"
        desc += f"Knee: {knee_amp:.1f}° amplitude, {knee_offset:.1f}° offset, {knee_phase:.2f} rad phase\n"
        desc += f"Frequency: {freq:.2f} Hz"
        return desc
    
    def _print_params(self, params):
        """Print parameters in a readable format"""
        names = ["Hip Amp", "Hip Off", "Hip Phase", 
                 "Knee Amp", "Knee Off", "Knee Phase",
                 "Freq", "Hip Kp", "Hip Kd", "Knee Kp", "Knee Kd"]
        
        for name, value in zip(names, params):
            if "Phase" in name:
                print(f"  {name:10s}: {value:.3f} rad")
            elif "Freq" in name:
                print(f"  {name:10s}: {value:.2f} Hz")
            elif "Kp" in name or "Kd" in name:
                print(f"  {name:10s}: {value:.1f}")
            else:
                print(f"  {name:10s}: {value:.2f}°")
    
    def validate_best(self, render=True):
        """Validate the best found parameters with visualization"""
        if self.best_params is None:
            print("No best parameters found yet. Run optimization first.")
            return
        
        print("\n" + "=" * 60)
        print("VALIDATING BEST PARAMETERS")
        print("=" * 60)
        self._print_params(self.best_params)
        
        cot = self.simulate_with_params(self.best_params, render=render)
        print(f"\nValidated COT: {cot:.4f}")
        
        return cot
    
    def plot_convergence(self):
        """Plot optimization convergence"""
        if not self.generation_history:
            print("No optimization history to plot")
            return
        
        plt.figure(figsize=(10, 6))
        plt.plot(self.generation_history, self.best_cot_history, 'b-', label='Best COT', linewidth=2)
        plt.plot(self.generation_history, self.avg_cot_history, 'r--', label='Average COT', linewidth=2)
        plt.xlabel('Generation')
        plt.ylabel('Cost of Transport')
        plt.title('Genetic Algorithm Convergence')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.yscale('log')  # Log scale to better see improvements
        plt.tight_layout()
        plt.show()


def run_optimization():
    """Main function to run the optimization"""
    
    # Create optimizer
    optimizer = COTOptimizer(
        xml_path="DOMS/tutorials/two_link_hopper_2_dof.xml",
        sim_duration=3.0  # Shorter duration for faster optimization
    )
    
    # Run genetic algorithm
    best_params, best_cot = optimizer.optimize(
        population_size=40,    # Number of individuals per generation
        generations=25,        # Number of generations
        mutation_rate=0.15,    # Mutation probability
        crossover_rate=0.8     # Crossover probability
    )
    
    # Plot convergence
    optimizer.plot_convergence()
    
    # Validate best parameters with visualization
    print("\nRunning validation simulation with best parameters...")
    optimizer.validate_best(render=True)
    
    return optimizer


def test_manual_parameters():
    """Test with manually tuned parameters for comparison"""
    print("\n" + "=" * 60)
    print("TESTING MANUAL PARAMETERS")
    print("=" * 60)
    
    # Manual parameters from original code
    manual_params = [
        30.0,    # hip_amp
        -10.0,   # hip_offset
        0.0,     # hip_phase
        45.0,    # knee_amp
        10.0,    # knee_offset
        np.pi,   # knee_phase
        2.0,     # frequency
        150.0,   # hip_kp
        15.0,    # hip_kd
        150.0,   # knee_kp
        15.0     # knee_kd
    ]
    
    optimizer = COTOptimizer("DOMS/tutorials/two_link_hopper_2_dof.xml")
    
    print("Parameters:")
    optimizer._print_params(manual_params)
    
    cot = optimizer.simulate_with_params(manual_params, render=True)
    print(f"\nCost of Transport: {cot:.4f}")
    
    return cot


if __name__ == "__main__":
    # Run optimization
    optimizer = run_optimization()
    
    # Optional: Compare with manual parameters
    print("\n" + "=" * 60)
    print("COMPARISON WITH MANUAL PARAMETERS")
    print("=" * 60)
    manual_cot = test_manual_parameters()
    
    print(f"\nOptimized COT: {optimizer.best_cot:.4f}")
    print(f"Manual COT: {manual_cot:.4f}")
    print(f"Improvement: {(manual_cot - optimizer.best_cot) / manual_cot * 100:.1f}%")