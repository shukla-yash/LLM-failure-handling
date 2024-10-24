import random

# Step 1: Initialize belief states and likelihoods

# Initial belief about reachability
P_reachable_obj1 = 0.8  # P(Object 1 is reachable)
P_reachable_obj2 = 0.6  # P(Object 2 is reachable)

# Initial uniform likelihood of success
P_success_given_reachable_obj1 = 0.5 # sensors - hardcoded info
P_success_given_not_reachable_obj1 = 0.5

# Adjust likelihood based on reachability of object 2
P_success_given_reachable_obj2 = 0.5
P_success_given_not_reachable_obj2 = 0.5

# Step 2: Define the total number of trials
num_trials = 1000

# Step 3: Loop over each trial to update beliefs based on outcomes
for trial in range(num_trials):
    
    # 1. Sample from prior beliefs: whether object 1 and object 2 are reachable
    obj1_reachable = random.random() < P_reachable_obj1
    obj2_reachable = random.random() < P_reachable_obj2
    
    # 2. Apply the pick(obj1) operator: determine if the action succeeds or fails
    
    # Compute likelihood of success based on the reachability of both objects
    if obj1_reachable:
        likelihood_success = P_success_given_reachable_obj1
    else:
        likelihood_success = P_success_given_not_reachable_obj1
    
    # Adjust likelihood based on the state of object 2
    if obj2_reachable:
        likelihood_success += 0.0  # Increase success probability if obj2 is reachable
    else:
        likelihood_success -= 0.0  # Decrease success probability if obj2 is not reachable

    # Ensure likelihood remains between 0 and 1
    likelihood_success = max(0, min(likelihood_success, 1))

    # 3. Simulate whether the action succeeds or fails
    action_success = random.random() < likelihood_success
    
    # 4. Update beliefs based on success or failure using Bayesian updating
    
    # Compute the total probability of success
    P_success = (
        P_success_given_reachable_obj1 * P_reachable_obj1 + 
        P_success_given_not_reachable_obj1 * (1 - P_reachable_obj1)
    )
    
    # Update beliefs for object 1
    if action_success:
        P_reachable_obj1 = (
            P_success_given_reachable_obj1 * P_reachable_obj1
        ) / P_success
    else:
        P_reachable_obj1 = (
            (1 - P_success_given_reachable_obj1) * P_reachable_obj1
        ) / (1 - P_success)
    
    # Update beliefs for object 2 similarly
    if action_success:
        P_reachable_obj2 = (
            P_success_given_reachable_obj2 * P_reachable_obj2
        ) / P_success
    else:
        P_reachable_obj2 = (
            (1 - P_success_given_reachable_obj2) * P_reachable_obj2
        ) / (1 - P_success)
    
    # Output the results of the trial
    print(f"Trial {trial + 1}: Success={action_success}, "
          f"P(Object 1 Reachable)={P_reachable_obj1:.4f}, "
          f"P(Object 2 Reachable)={P_reachable_obj2:.4f}")
    
    # Adjust the likelihood for future trials based on evidence
    if action_success:
        # If the action succeeds, increase the likelihood of success
        P_success_given_reachable_obj1 = min(P_success_given_reachable_obj1 + 0.01, 1)
        P_success_given_reachable_obj2 = min(P_success_given_reachable_obj2 + 0.01, 1)
    else:
        # If the action fails, decrease the likelihood of success
        P_success_given_reachable_obj1 = max(P_success_given_reachable_obj1 - 0.01, 0)
        P_success_given_reachable_obj2 = max(P_success_given_reachable_obj2 - 0.01, 0)
