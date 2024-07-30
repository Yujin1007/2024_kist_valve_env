import numpy as np
import argparse
import json
import os
import copy
from tqc import structures, DEVICE
from tqc.trainer import Trainer
from tqc.structures import Actor, Critic
import fr3Env
# from Classifier import Classifier

def main(PATH, TRAIN, RENDERING, OBJ, OFFLINE1, OFFLINE2):
    env = fr3Env.valve_env1(RENDERING, TRAIN, OBJ)
    env.env_rand = False

    max_timesteps = 1e6
    max_episode = 1e4
    batch_size = 16
    policy_kwargs = dict(n_critics=5, n_quantiles=25)
    save_freq = 1e5
    models_dir = PATH
    pretrained_model_dir = models_dir# + "10.0/"
    episode_data = []
    timestep_data = []
    save_flag = False

    state_dim = env.observation_space.shapes
    action_dim1 = env.rotation_action_space.shape[0]
    action_dim2 = env.force_action_space.shape[0]

    replay_buffer1 = structures.ReplayBuffer((env.len_hist,state_dim), action_dim1)
    actor1 = Actor(state_dim, action_dim1, IsDemo=OFFLINE1).to(DEVICE)

    critic1 = Critic(state_dim, action_dim1, policy_kwargs["n_quantiles"], policy_kwargs["n_critics"], IsDemo=OFFLINE1).to(DEVICE)
    critic_target1 = copy.deepcopy(critic1)


    replay_buffer2 = structures.ReplayBuffer((env.len_hist,state_dim), action_dim2)
    actor2 = Actor(state_dim, action_dim2, IsDemo=OFFLINE2).to(DEVICE)

    critic2 = Critic(state_dim, action_dim2, policy_kwargs["n_quantiles"], policy_kwargs["n_critics"], IsDemo=OFFLINE2).to(DEVICE)
    critic_target2 = copy.deepcopy(critic2)

    demo_path1 = None
    demo_path2 = None
    if TRAIN:
        if OFFLINE1:
            demo_path1 = "path_to_replay_buffer_rotation.pkl"
        if OFFLINE2:
            demo_path2 = "path_to_replay_buffer_force.pkl"

    trainer1 = Trainer(actor=actor1,
                       critic=critic1,
                       critic_target=critic_target1,
                       top_quantiles_to_drop=2,
                       discount=0.99,
                       tau=0.005,
                       target_entropy=-np.prod(env.rotation_action_space.shape).item(),
                        demo_path=demo_path1)

    trainer2 = Trainer(actor=actor2,
                       critic=critic2,
                       critic_target=critic_target2,
                       top_quantiles_to_drop=2,
                       discount=0.99,
                       tau=0.005,
                       target_entropy=-np.prod(env.force_action_space.shape).item(),
                       demo_path=demo_path2)


    episode_return_rotation = 0
    episode_return_force = 0
    episode_timesteps = 0
    episode_num = 0
    episode_return_rotation_accum = 0
    episode_return_force_accum = 0
    episode_cnt = 0

    if TRAIN:
        state = env.reset()

        actor1.train()
        actor2.train()
        for t in range(int(max_timesteps)):

            action_rotation = actor1.select_action(state)
            action_force = actor2.select_action(state)


            next_state, reward_rotation, reward_force, done, _ = env.step(action_rotation, action_force)

            episode_timesteps += 1

            replay_buffer1.add(state, action_rotation, next_state, reward_rotation, done)
            replay_buffer2.add(state, action_force, next_state, reward_force, done)

            state = next_state
            episode_return_rotation += reward_rotation
            episode_return_force += reward_force

            # Train agent after collecting sufficient data
            if t >= batch_size:
                trainer1.train(replay_buffer1, batch_size)
                trainer2.train(replay_buffer2, batch_size)
            if (t + 1) % save_freq == 0:
                save_flag = True
            if done:
                # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
                print(
                    f"Total T: {t + 1} Episode Num: {episode_num + 1} Episode T: {episode_timesteps} "
                    f"Reward R: {episode_return_rotation:.3f} Reward F: {episode_return_force:.3f}")


                # Reset environment
                state = env.reset()
                episode_data.append([episode_num, episode_timesteps, episode_return_rotation, episode_return_force])
                episode_return_rotation_accum += episode_return_rotation
                episode_return_force_accum += episode_return_force
                episode_cnt += 1

                episode_return_rotation = 0
                episode_return_force = 0
                episode_timesteps = 0
                episode_num += 1
            if (t+1) % 10000 == 0:
                timestep_data.append([episode_return_rotation_accum / episode_cnt, episode_return_force_accum / episode_cnt])
                episode_return_rotation_accum = 0
                episode_return_force_accum = 0
                episode_cnt = 0
                np.save(models_dir + "avg_reward.npy", timestep_data)
            if save_flag:
                path1 = models_dir + str((t + 1) // save_freq) + "/rotation/"
                path2 = models_dir + str((t + 1) // save_freq) + "/force/"
                os.makedirs(path1, exist_ok=True)
                os.makedirs(path2, exist_ok=True)
                if not os.path.exists(path1):
                    os.makedirs(path1)
                if not os.path.exists(path2):
                    os.makedirs(path2)
                trainer1.save(path1)
                trainer2.save(path2)

                np.save(models_dir + "reward.npy", episode_data)
                save_flag = False

    else:
        pretrained_model_dir1 = pretrained_model_dir+"rotation/"
        pretrained_model_dir2 = pretrained_model_dir+"force/"
        trainer1.load(pretrained_model_dir1)
        actor1.eval()
        critic1.eval()
        actor1.training = False

        trainer2.load(pretrained_model_dir2)
        actor2.eval()
        critic2.eval()
        actor2.training = False
        num_ep = 16
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
                # force_data.append(env.force.copy())
                state = next_state
                episode_return_rotation += reward_rotation
                episode_return_force += reward_force

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--description", help="description")
    parser.add_argument("--path", help="data load path", default=" ./log/0527_1/")
    parser.add_argument("--render", help="0->no rendering,  1->rendering", type=int, default=0)
    parser.add_argument("--train", help="0->test, 1->train", type=int, default=1)
    parser.add_argument("--obj", help="valve or handle", default="handle")
    parser.add_argument("--offline1", help="0->false, 1->true",type=int, default=1)
    parser.add_argument("--offline2", help="0->false, 1->true", type=int,default=0)
    args = parser.parse_args()
    args_dict = vars(args)
    if args.train == 1:
        os.makedirs(args.path, exist_ok=True)
        if not os.path.exists(args.path):
            os.makedirs(args.path)
        with open(args.path + 'args_and_notes.json', 'w') as f:
            json.dump(args_dict, f, indent=4)
    # Print Arguments
    print("------------ Arguments -------------")
    for key, value in vars(args).items():
        print(f"{key} : {value}")
    print("------------------------------------")

    main(PATH=args.path, TRAIN=args.train, RENDERING=args.render, OBJ=args.obj, OFFLINE1=args.offline1, OFFLINE2=args.offline2)
    '''
    handle test :
    python main.py --train 0 --obj handle --render 1 --offline1 1 --offline2 0 --path ../../weight/Handle/
    handle train : 
    python main.py --train 1 --obj handle  --render 0 --offline1 0 --offline2 0 --path "path to my dir" # without expert demonstration
    python main.py --train 1 --obj handle --render 0 --offline1 1 --offline2 1 --path "path to my dir" # with expert demonstration 
    
    valve test :
    python main.py --train 0 --obj valve --render 1 --offline1 0 --offline2 0 --path ../../weight/Valve/
    handle train : 
    python main.py --train 1 --obj valve  --render 0 --offline1 0 --offline2 0 --path "path to my dir" # without expert demonstration
    python main.py --train 1 --obj valve --render 0 --offline1 1 --offline2 1 --path "path to my dir" # with expert demonstration 
    
    '''