import numpy as np


def make_sample_her_transitions(replay_strategy, replay_k, reward_fun):
    """Creates a sample function that can be used for HER experience replay.

    Args:
        replay_strategy (in ['future', 'none']): the HER replay strategy; if set to 'none',
            regular DDPG experience replay is used
        replay_k (int): the ratio between HER replays and regular replays (e.g. k = 4 -> 4 times
            as many HER replays as regular replays are used)
        reward_fun (function): function to re-compute the reward with substituted goals
    """
    if replay_strategy == 'future':
        future_p = 1 - (1. / (1 + replay_k))
    else:  # 'replay_strategy' == 'none'
        future_p = 0

    def _sample_her_transitions(episode_batch, batch_size_in_transitions):
        """episode_batch is {key: array(buffer_size x T x dim_key)}
        """
        # print("EPISODE BATCH U")
        # print(episode_batch['ag'].shape)
        # print(episode_batch['g'].shape)
        # print(episode_batch['o_2'].shape)
        # print(episode_batch['o'].shape)
        # print(episode_batch['ag_2'].shape)
        # print(episode_batch['u'].shape)
        # print(episode_batch.keys())

        #for key in episode_batch.keys():
        #    if episode_batch[key][0].shape[0] == 1:
        #        print("Shape == 1")
        #        print(key,episode_batch[key][0])
        #        episode_batch[key] = (episode_batch[key][0][1], episode_batch[key][0][2],)
            #else:
                #print("SHAPE != 1")
                #print(key,episode_batch[key])
                #episode_batch[key] = (episode_batch[key][0][0],episode_batch[key][0][1])

        # print("EPISODE BATCH U")
        # print(episode_batch['ag'])
        # print(episode_batch['o_2'])
        # print(episode_batch['o'])
        # print(episode_batch['ag_2'])
        # print(episode_batch['u'])
        #T = episode_batch['u'][0].shape[0]
        T = episode_batch['u'][0].shape[1]
        #rollout_batch_size = episode_batch['u'][0].shape[0]
        rollout_batch_size = episode_batch['u'].shape[0]
        batch_size = batch_size_in_transitions

        # Select which episodes and time steps to use.
        episode_idxs = np.random.randint(0, rollout_batch_size, batch_size)
        t_samples = np.random.randint(T, size=batch_size)
        #print(episode_idxs)
        # print(rollout_batch_size)
        # print(batch_size)
        # print(t_samples)
        # print(T)
        #print(episode_batch[key][0])
        #print(episode_batch['ag'][0][episode_idxs])
        #print(episode_batch['ag'][0][episode_idxs, t_samples])
        #transitions = {key: episode_batch[key][0][episode_idxs, t_samples].copy()
        transitions = {key: episode_batch[key][episode_idxs, t_samples].copy()
                       for key in episode_batch.keys()}

        # Select future time indexes proportional with probability future_p. These
        # will be used for HER replay by substituting in future goals.
        her_indexes = np.where(np.random.uniform(size=batch_size) < future_p)
        future_offset = np.random.uniform(size=batch_size) * (T - t_samples)
        future_offset = future_offset.astype(int)
        #future_t = (t_samples + 1 + future_offset)[her_indexes]
        future_t = (t_samples + future_offset)[her_indexes]

        # Replace goal with achieved goal but only for the previously-selected
        # HER transitions (as defined by her_indexes). For the other transitions,
        # keep the original goal.
        # print("INDEX")
        # print(episode_idxs[her_indexes])
        # print(future_t)
        #future_ag = episode_batch['ag'][0][episode_idxs[her_indexes], future_t]
        future_ag = episode_batch['ag'][episode_idxs[her_indexes], future_t]
        # print(future_ag)
        transitions['g'][her_indexes] = future_ag

        # Reconstruct info dictionary for reward  computation.
        info = {}
        for key, value in transitions.items():
            if key.startswith('info_'):
                info[key.replace('info_', '')] = value

        # Re-compute reward since we may have substituted the goal.
        reward_params = {k: transitions[k] for k in ['ag_2', 'g']}
        reward_params['info'] = info
        transitions['r'] = reward_fun(**reward_params)

        #print("BATCH SIZE")
        #print(batch_size)
        #print("TRANSITIONS")
        #print(transitions['g'].shape)
        #print(transitions['ag_2'].shape)
        #print(transitions['u'].shape)
        #transitions = {k: transitions[k].reshape(batch_size, *transitions[k].shape[1:])
                       #for k in transitions.keys()}

        assert(transitions['u'].shape[0] == batch_size_in_transitions)

        return transitions

    return _sample_her_transitions
