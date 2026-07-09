import random
import optuna
import json
from controller import Supervisor

# SETUP AND CONSTANTS
supervisor = Supervisor()
root_node = supervisor.getRoot()
children_field = root_node.getField('children')
timestep = int(supervisor.getBasicTimeStep())

EPUCK_DIA = 0.074
HOLE_WIDTH = EPUCK_DIA * 1.5  
ARENA_Y = 2.0  
WALL_X = 0.0   

spawned_nodes = [] 

# SCENARIOS

all_scenarios = []
for r in range(6, 11):      # robots
    for h in range(2, 5):   # holes
        # Difficulty -> Robots per Hole 
        difficulty = r / h 
        all_scenarios.append({"robots": r, "holes": h, "difficulty": difficulty})
# Hardest scenarios first 
all_scenarios.sort(key=lambda x: x["difficulty"], reverse=True)

# OPTUNA OBJECTIVE FUNCTION
def objective(trial):
    global spawned_nodes

    # Hyperparams range
    w_light = trial.suggest_float("w_light", 1e-5, 1e-3, log=True)
    w_prox = trial.suggest_float("w_prox", 1e-6, 1e-4, log=True)
    w_slide = trial.suggest_float("w_slide", 1e-3, 5e-2, log=True)
    w_noise = trial.suggest_float("w_noise", 0.1, 3.0)
    
    custom_data_str = f"{w_light},{w_prox},{w_slide},{w_noise}"
    
    total_score = 0
    print(f"\n--- Starting Trial {trial.number} ---")

    for step, scenario in enumerate(all_scenarios):
        num_robots = scenario["robots"]
        num_holes = scenario["holes"]
        
        # Previous world cleanup
        for node in spawned_nodes:
            try:
                node.remove()
            except:
                pass
        spawned_nodes.clear()

        # Spwan world
        for i in range(num_robots):
            x = random.uniform(-1.4, -0.3)
            y = random.uniform(-0.9, 0.9)
            rot = random.uniform(0, 6.28)
            
            epuck_str = f'''
            DEF EPUCK_{i} E-puck {{
              translation {x} {y} 0
              rotation 0 0 1 {rot}
              name "epuck_{i}"
              controller "fluid_swarm"
              customData "{custom_data_str}" 
            }}
            '''
            children_field.importMFNodeFromString(-1, epuck_str)
            spawned_nodes.append(supervisor.getFromDef(f"EPUCK_{i}"))

        total_solid_space = ARENA_Y - (num_holes * HOLE_WIDTH)
        segment_length = total_solid_space / (num_holes + 1)
        current_y = -1.0 

        for i in range(num_holes + 1):
            center_y = current_y + (segment_length / 2)
            wall_str = f'''
            DEF WALL_{i} Solid {{
              translation {WALL_X} {center_y} 0.05
              children [
                Shape {{
                  appearance PBRAppearance {{ baseColor 0.2 0.2 0.8 roughness 1 metalness 0 }}
                  geometry Box {{ size 0.1 {segment_length} 0.2 }}
                }}
              ]
              boundingObject Box {{ size 0.1 {segment_length} 0.2 }}
            }}
            '''
            children_field.importMFNodeFromString(-1, wall_str)
            spawned_nodes.append(supervisor.getFromDef(f"WALL_{i}"))
            current_y += segment_length + HOLE_WIDTH

        # Run sub-simulation
        MAX_SECONDS = 120  # 2 minutes max to clear the wall (TODO: 1.5 or even sub 1 min for harder scenarios)
        max_ticks = int((MAX_SECONDS * 1000) / timestep)
        ticks = 0
        finished_count = 0
        
        while supervisor.step(timestep) != -1 and ticks < max_ticks:
            ticks += 1
            finished_count = 0
            
            for i in range(num_robots):
                node = spawned_nodes[i] 
                if node is not None:
                    pos = node.getPosition()
                    if pos[0] > 1.0: 
                        finished_count += 1
                        
            if finished_count == num_robots:
                break 

        # SCORING
        time_taken = ticks * (timestep / 1000.0)
        failed_robots = num_robots - finished_count
        
        sub_score = time_taken + (failed_robots * 50) 
        total_score += sub_score
        #TODO: Change it so it shows the the current simulation result when it finishes and not the previous' one
        print(f"  Step {step+1}/15 ({num_robots}R/{num_holes}H) | Time: {time_taken:.1f}s | Failed: {failed_robots}")

        # Kill switch: If even ONE robot fails to cross, PRUNE the trial!
        if failed_robots > 0:
            remaining_scenarios = len(all_scenarios) - (step + 1)
            penalty = remaining_scenarios * 300  # Massive penalty for the skipped scenarios
            total_score += penalty
            
            print(f"  [EARLY STOP] Parameters failed the Gauntlet. Pruning {remaining_scenarios} scenarios. Penalty applied.")
            
            # Inform Optuna that this trial was pruned so it learns faster
            raise optuna.TrialPruned()

    print(f"Trial {trial.number} CLEARED THE GAUNTLET! -> Total Score: {total_score:.2f}")
    return total_score

# OPTUNA STUDY
print("STARTING FAIL-FAST OPTUNA PIPELINE")

# MedianPruner discards bad weights early
study = optuna.create_study(direction="minimize", pruner=optuna.pruners.MedianPruner())

study.optimize(objective, n_trials=200)

print("\n")
print("OPTIMIZATION COMPLETE!")
print("Best Generalized Score: ", study.best_value)
print("Best Universal Parameters:")
for key, value in study.best_params.items():
    print(f"  {key}: {value}")

print("\nExporting results to JSON...")

results_export = {
    "best_score": study.best_value,
    "best_params": study.best_params,
    "all_trials": []
}

for t in study.trials:
    trial_data = {
        "trial_number": t.number,
        "state": str(t.state),              # 'TrialState.COMPLETE' or 'TrialState.PRUNED'
        "final_score": t.value,
        "params": t.params,
        "history": t.user_attrs.get("step_history", [])
    }
    results_export["all_trials"].append(trial_data)

json_filename = "optuna_swarm_results.json"
with open(json_filename, "w") as f:
    json.dump(results_export, f, indent=4)

print(f"Saved to '{json_filename}'")

supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)