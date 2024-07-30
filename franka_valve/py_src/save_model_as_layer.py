import numpy as np
import torch
import gym
import argparse
import os
import copy
from pathlib import Path
import csv
from tqc import structures, DEVICE
from tqc.trainer import Trainer
from tqc.structures import Actor, Critic, Actor2
from tqc.functions import eval_policy
# from fr3Env import fr3_smooth_start
import fr3Env
import pandas as pd
import torch.nn as nn
import torch.nn.functional as F
from Classifier import Classifier
import matplotlib.pyplot as plt
# class Classifier(nn.Module):
#     def __init__(self, input_size, output_size):
#         super(Classifier, self).__init__()
#         self.fc1 = nn.Linear(input_size, 64)
#         self.fc2 = nn.Linear(64, 256)
#         self.fc_ = nn.Linear(256, 256)
#         self.fc4 = nn.Linear(256, output_size)
#
#     def forward(self, x):
#         x = F.relu(self.fc1(x))
#         x = F.relu(self.fc2(x))
#         x = F.relu(self.fc_(x))
#         x = F.relu(self.fc4(x))
#         return x
def save_lstm_weights_and_biases_in_parts(param, gate_names, directory, base_filename):
    param_np = param.detach().cpu().numpy()
    split_params = torch.chunk(torch.tensor(param_np), 4, dim=0)

    for gate_name, split_param in zip(gate_names, split_params):
        split_param_df = pd.DataFrame(split_param.numpy())
        filename = os.path.join(directory, f"{base_filename}_{gate_name}.csv")
        split_param_df.to_csv(filename, header=False, index=False)
        print(f"Saved {base_filename}_{gate_name} to {filename}")

def save_model_to_csv(model, directory="model_params"):
    if not os.path.exists(directory):
        os.makedirs(directory)

    gate_names = ['input', 'forget', 'cell', 'output']

    for name, param in model.named_parameters():
        base_filename = name.replace('.', '_')
        if 'weight_ih_l0' in name or 'weight_hh_l0' in name or 'bias_ih_l0' in name or 'bias_hh_l0' in name:
            save_lstm_weights_and_biases_in_parts(param, gate_names, directory, base_filename)
        else:
            param_np = param.detach().cpu().numpy()
            param_df = pd.DataFrame(param_np)
            filename = os.path.join(directory, base_filename + '.csv')
            param_df.to_csv(filename, header=False, index=False)
            print(f"Saved {name} to {filename}")
# def load_model_from_csv(model, directory="model_params"):
#     for name, param in model.named_parameters():
#         filename = os.path.join(directory, name.replace('.', '_') + '.csv')
#         param_df = pd.read_csv(filename, header=None)
#         param_data = torch.tensor(param_df.values, dtype=param.dtype)
#         print(name, param.shape, param_data.shape)
#
#         if param_data.shape == param.shape:
#             pass
#         else:
#             param_data = param_data.reshape(param.shape)
#         param.data.copy_(param_data)
#         print(f"Loaded {name} from {filename}")
def load_lstm_weights_and_biases_from_parts(directory, base_filename, gate_names):
    parts = []
    for gate_name in gate_names:
        filename = os.path.join(directory, f"{base_filename}_{gate_name}.csv")
        part_df = pd.read_csv(filename, header=None)
        part_tensor = torch.tensor(part_df.values, dtype=torch.float32)
        parts.append(part_tensor)

    # Concatenate the parts along the 0th dimension (rows)
    full_param = torch.cat(parts, dim=0)
    return full_param


def load_model_from_csv(model, directory="model_params"):
    gate_names = ['input', 'forget', 'cell', 'output']

    for name, param in model.named_parameters():
        base_filename = name.replace('.', '_')
        if 'weight_ih_l0' in name or 'weight_hh_l0' in name or 'bias_ih_l0' in name or 'bias_hh_l0' in name:
            full_param = load_lstm_weights_and_biases_from_parts(directory, base_filename, gate_names)
            if full_param.shape != param.shape:
                full_param = full_param.reshape(param.shape)
            param.data.copy_(full_param)
        else:
            filename = os.path.join(directory, base_filename + '.csv')
            param_df = pd.read_csv(filename, header=None)
            param_tensor = torch.tensor(param_df.values, dtype=torch.float32)
            if param_tensor.shape != param.shape:
                param_tensor = param_tensor.reshape(param.shape)
            param.data.copy_(param_tensor)
        # filename = os.path.join(directory, base_filename + '.csv')
        # param_df = pd.read_csv(filename, header=None)
        # param_tensor = torch.tensor(param_df.values, dtype=torch.float32)
        # if param_tensor.shape != param.shape:
        #     print(param_tensor.shape, param.shape)
        #     param_tensor = param_tensor.reshape(param.shape)
        # param.data.copy_(param_tensor)


def BringClassifier(path):
    classifier = torch.load(path)
    classifier.eval()
    return classifier
class Classifier(nn.Module):
    def __init__(self, input_size, output_size):
        super(Classifier, self).__init__()
        self.fc1 = nn.Linear(input_size, 256)
        self.fc2 = nn.Linear(256, 1024)
        self.fc3 = nn.Linear(1024,1024)
        # self.fc4 = nn.Linear(1024, 256)
        # self.fc5 = nn.Linear(256, output_size)
        self.fc_ = nn.Linear(1024, 256)
        self.fc4 = nn.Linear(256, output_size)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc_(x))

        x = F.softmax(self.fc4(x),dim=0)
        return x


def main():
    # --- Init ---
    SAVE = True
    # env = fr3_valve_size()
    env = fr3Env.real_robot(TRAIN=False, RENDERING=True,OBJ="handle")

    policy_kwargs = dict(n_critics=5, n_quantiles=25)
    # models_dir = "./log/new_loss_1/"
    pretrained_model_dir = "./log/noLSTM2/10.0/"
    pretrained_model_dir1 = pretrained_model_dir + "rotation/"
    pretrained_model_dir2 = pretrained_model_dir + "force/"
    # pretrained_model_dir1 = "./log/0625_1/10.0/rotation/"
    # pretrained_model_dir2 = "./log/0527_1/10.0/force/"
    weight_path = "./weight/"
    state_dim = env.observation_space.shapes
    action_dim1 = env.rotation_action_space.shape[0]
    action_dim2 = env.force_action_space.shape[0]

    replay_buffer1 = structures.ReplayBuffer((env.len_hist, state_dim), action_dim1, max_size=int(1e5))
    actor1 = Actor2(state_dim*env.len_hist, action_dim1).to(DEVICE)

    critic1 = Critic(state_dim, action_dim1, policy_kwargs["n_quantiles"], policy_kwargs["n_critics"]).to(
        DEVICE)
    critic_target1 = copy.deepcopy(critic1)

    trainer1 = Trainer(actor=actor1,
                       critic=critic1,
                       critic_target=critic_target1,
                       top_quantiles_to_drop=2,
                       discount=0.99,
                       tau=0.005,
                       target_entropy=-np.prod(env.rotation_action_space.shape).item())

    replay_buffer2 = structures.ReplayBuffer((env.len_hist, state_dim), action_dim2, max_size=int(1e5))
    actor2 = Actor2(state_dim*env.len_hist, action_dim2).to(DEVICE)

    critic2 = Critic(state_dim, action_dim2, policy_kwargs["n_quantiles"], policy_kwargs["n_critics"]).to(
        DEVICE)
    critic_target2 = copy.deepcopy(critic2)

    trainer2 = Trainer(actor=actor2,
                       critic=critic2,
                       critic_target=critic_target2,
                       top_quantiles_to_drop=2,
                       discount=0.99,
                       tau=0.005,
                       target_entropy=-np.prod(env.force_action_space.shape).item())
    if SAVE:
        trainer1.load(pretrained_model_dir1)
        trainer2.load(pretrained_model_dir2)
        classifier_clk = BringClassifier("./classifier/model_clk.pt")
        classifier_cclk = BringClassifier("./classifier/model_cclk.pt")

        save_model_to_csv(actor1, weight_path+"rotation")
        save_model_to_csv(actor2, weight_path+"force")
        save_model_to_csv(classifier_clk, weight_path+"classifier_clk")
        save_model_to_csv(classifier_cclk, weight_path+"classifier_cclk")
    else:
        classifier_clk = Classifier(input_size=8, output_size=20)
        classifier_cclk = Classifier(input_size=8, output_size=20)
        load_model_from_csv(actor1, weight_path + "rotation")
        load_model_from_csv(actor2, weight_path + "force")
        load_model_from_csv(classifier_clk, weight_path + "classifier_clk")
        load_model_from_csv(classifier_cclk, weight_path + "classifier_cclk")
        # load_model_from_csv(classifier_cclk, weight_path + "classifier_cclk")

        num_ep = 1000
        force_data = []
        # env.episode_number = 3
        # df = pd.read_csv("/home/kist-robot2/Downloads/obs_real.csv")
        # states = df.to_numpy(dtype=np.float32)
        env.env_rand = False
        for _ in range(num_ep):
            state = env.reset()
            done = False
            step_cnt = 0
            episode_return_rotation = 0
            episode_return_force = 0
            force_data = []
            while not done:
                step_cnt += 1
                action_rotation = actor1.select_action(state)
                action_force = actor2.select_action(state)
                next_state, reward_rotation, reward_force, done, _ = env.step(action_rotation, action_force)
                force_data.append(env.force)

                state = next_state
                episode_return_rotation += reward_rotation
                episode_return_force += reward_force

            # np.save("./data/torque_hybrid.npy", env.torque_data)
            # print(
            #     f"Total T: {t + 1} Episode Num: {episode_num + 1} Episode T: {episode_timesteps} "
            #     f"Reward R: {episode_return_rotation:.3f} Reward F: {episode_return_force:.3f}")
            print(
                f"Reward R: {episode_return_rotation:.3f} Reward F: {episode_return_force:.3f}")
            print("time:", env.time_done, "  deviation:", env.deviation_done, "  bound:", env.bound_done,
                  "  goal:", env.goal_done)
            plt.plot(env.force_profile_data, linestyle='-', color='b')
            plt.title(env.friction)
            plt.show()

    # tmp = copy.deepcopy(actor.net.fc0.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/w1.csv", header=False, index=False)
    # tmp = copy.deepcopy(actor.net.fc0.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/b1.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(actor.net.fc1.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/w2.csv", header=False, index=False)
    # tmp = copy.deepcopy(actor.net.fc1.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/b2.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(actor.net.last_fc.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/w3.csv", header=False, index=False)
    # tmp = copy.deepcopy(actor.net.last_fc.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/actor/b3.csv", header=False, index=False)
    #
    #
    #
    #
    # tmp = copy.deepcopy(env.classifier_cclk.fc1.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/w1.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_cclk.fc1.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/b1.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_cclk.fc2.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/w2.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_cclk.fc2.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/b2.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_cclk.fc_.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/w3.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_cclk.fc_.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/b3.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_cclk.fc4.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/w4.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_cclk.fc4.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_cclk/b4.csv", header=False, index=False)
    #
    #
    # tmp = copy.deepcopy(env.classifier_clk.fc1.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/w1.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_clk.fc1.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/b1.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_clk.fc2.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/w2.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_clk.fc2.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/b2.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_clk.fc_.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/w3.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_clk.fc_.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/b3.csv", header=False, index=False)
    #
    # tmp = copy.deepcopy(env.classifier_clk.fc4.weight)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/w4.csv", header=False, index=False)
    # tmp = copy.deepcopy(env.classifier_clk.fc4.bias)
    # tmp = tmp.detach().cpu().numpy()
    # pd.DataFrame(tmp).to_csv("./weight/classifier_clk/b4.csv", header=False, index=False)
    #
    #
    # ## test saved weights
    # # fc1_b = torch.load("./weight/b1")
    # # fc1_w = torch.load("./weight/w1")
    # # fc2_b = torch.load("./weight/b2")
    # # fc2_w = torch.load("./weight/w2")
    # # fc_mu_b = torch.load("./weight/b3")
    # # fc_mu_w = torch.load("./weight/w3")
    # #
    # # actor.net.fc0.weight =fc1_w
    # # actor.net.fc0.bias=fc1_b
    # # actor.net.fc1.weight=fc2_w
    # # actor.net.fc1.bias=fc2_b
    # # actor.net.last_fc.weight=fc_mu_w
    # # actor.net.last_fc.bias=fc_mu_b
    # #
    # # actor.net.fcs[0].weight=fc1_w
    # # actor.net.fcs[0].bias=fc1_b
    # # actor.net.fcs[1].weight=fc2_w
    # # actor.net.fcs[1].bias=fc2_b
    #
    # numbers_array = []
    #
    # # Path to your CSV file
    # csv_file = '/home/kist-robot2/Downloads/new_code/clik_obs_static.csv'
    #
    # # Open the CSV file and read line by line
    # with open(csv_file, newline='') as file:
    #     reader = csv.reader(file)
    #     for row in reader:
    #         # Convert each element in the row to integers and append to the list
    #         row = row[:91]
    #         numbers_array.append([float(num) for num in row])
    #
    # episode_return = 0
    #
    # actor.eval()
    # critic.eval()
    # actor.training = False
    # # reset_agent.training = False
    # num_ep = 5
    #
    # # print(actor.select_action(numbers_array[0]))
    # # print("0.48236	-1.69255	3.66839	-0.984286	2.45705	-3.29748")
    # for _ in range(num_ep):
    #     state = env.reset()
    #     done = False
    #     while not done:
    #         action = actor.select_action(state)
    #
    #         next_state, reward, done, _ = env.step(action)
    #         state = next_state
    #         episode_return += reward
    #     print("episode :", env.episode_number, "goal angle :", env.required_angle, "handle angle :", env.handle_angle)
    #
    #     print("time:",env.time_done, "  contact:",env.contact_done, "  bound:",env.bound_done,
    #           "  goal:", env.goal_done, "  reset:",env.reset_done)

if __name__ == "__main__":
    main()
