import torch
import torch.nn as nn
import torch.optim as optim
import cvxpy as cp
from cvxpylayers.torch import CvxpyLayer
import numpy as np

torch.manual_seed(42)
np.random.seed(42)

# Set default tensor type to double for compatibility with cvxpylayers which uses float64
torch.set_default_dtype(torch.float64)

# --- 1. Problem Setup ---
# Ground Wheeled Vehicle (Linearized 2D Kinematics for MPC)
n_state = 2
n_ctrl = 2
horizon = 5
dt = 0.1

A_mat = np.eye(n_state)
B_mat = np.eye(n_state) * dt

# True Complex Environment (Ground truth dynamics)
def true_dynamics(state, action):
    vx, vy = action[0], action[1]

    # Simulate wheel slip (non-linear)
    # The true dynamics: actual speed is smaller than commanded speed
    slip_vx = vx * 0.8
    slip_vy = vy * 0.8

    next_x = state[0] + slip_vx * dt
    next_y = state[1] + slip_vy * dt
    return torch.stack([next_x, next_y])

# --- 2. Neural Network (Residual Dynamics) ---
class ResidualNet(nn.Module):
    def __init__(self):
        super().__init__()
        # Input: state, action. Output: residual change in state
        self.net = nn.Sequential(
            nn.Linear(n_state + n_ctrl, 32),
            nn.Tanh(), # Tanh is smooth and twice differentiable
            nn.Linear(32, 32),
            nn.Tanh(),
            nn.Linear(32, n_state)
        )

    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        return self.net(x)

# --- 3. Differentiable MPC using CvxpyLayer ---
def build_mpc_layer():
    z = cp.Variable((horizon + 1, n_state))
    u = cp.Variable((horizon, n_ctrl))

    z_init = cp.Parameter(n_state)
    target = cp.Parameter(n_state)
    c_pred = cp.Parameter((horizon, n_state))

    cost = 0
    constraints = [z[0] == z_init]

    for t in range(horizon):
        cost += 10.0 * cp.sum_squares(z[t+1] - target)
        cost += 0.1 * cp.sum_squares(u[t])

        # Dynamics constraint: Nominal + NN Residual (c_pred)
        constraints += [z[t+1] == A_mat @ z[t] + B_mat @ u[t] + c_pred[t]]

        # Control constraints
        constraints += [u[t] <= 10.0, u[t] >= -10.0]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    mpc_layer = CvxpyLayer(prob, parameters=[z_init, target, c_pred], variables=[z, u])
    return mpc_layer

# --- 4. Training Loop ---
def train():
    mpc_layer = build_mpc_layer()
    nn_model = ResidualNet()
    optimizer = optim.Adam(nn_model.parameters(), lr=0.01)

    epochs = 50
    sim_steps = 20

    print("Starting End-to-End Training for Ground Vehicle MPC...")

    for epoch in range(epochs):
        optimizer.zero_grad()

        state = torch.tensor([0.0, 0.0])
        target = torch.tensor([5.0, 5.0])

        total_loss = 0

        # To accumulate trajectory for debugging
        traj = [state]

        for step in range(sim_steps):
            # Evaluate NN at current state with zero action to get residual prediction
            dummy_action = torch.zeros(n_ctrl)
            res_pred = nn_model(state, dummy_action)

            c_pred_seq = res_pred.unsqueeze(0).repeat(1, horizon, 1)

            z_init_batched = state.unsqueeze(0)
            target_batched = target.unsqueeze(0)

            try:
                z_opt, u_opt = mpc_layer(z_init_batched, target_batched, c_pred_seq)
            except Exception as e:
                print("Solver failed:", e)
                break

            action = u_opt[0, 0, :]

            # Step real dynamics
            next_state = true_dynamics(state, action)

            # Distance loss
            step_loss = torch.sum((next_state - target)**2)
            total_loss += step_loss

            state = next_state
            traj.append(state)

        total_loss.backward()
        torch.nn.utils.clip_grad_norm_(nn_model.parameters(), 1.0)
        optimizer.step()

        if epoch % 10 == 0 or epoch == epochs - 1:
            print(f"Epoch {epoch:2d} | Tracking Loss: {total_loss.item():.4f} | Final Position: [{state[0].item():.2f}, {state[1].item():.2f}]")

if __name__ == "__main__":
    train()
