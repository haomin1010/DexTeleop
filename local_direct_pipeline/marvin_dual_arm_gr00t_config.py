"""GR00T modality config for the Marvin dual-arm dexterous-hand dataset.

This config is designed to be reused across datasets that share the same:
- robot embodiment
- camera layout
- state/action slicing in ``meta/modality.json``
- language annotation key

Important:
- The default action horizon here is 6 because the current dataset's
  ``meta/relative_stats.json`` was generated for 6 steps.
- If you change the horizon, regenerate dataset statistics with
  ``python gr00t/data/stats.py`` before training.
"""

from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


ACTION_HORIZON = 6


marvin_dual_arm_config = {
    # Three RGB views: one head camera and two wrist cameras.
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=[
            "head",
            "left_wrist",
            "right_wrist",
        ],
    ),
    # Current proprioceptive state.
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=[
            "left_arm_joint",
            "right_arm_joint",
            "left_arm_ee_pose",
            "right_arm_ee_pose",
            "left_hand",
            "right_hand",
        ],
    ),
    # Action layout mirrors the dataset's action slices.
    #
    # Conservative defaults:
    # - arm joints: relative actions often generalize better
    # - eef poses: absolute pose targets, kept in DEFAULT format because the
    #   dataset stores 6 values (xyz + rotation vector / euler-like layout),
    #   not 9D xyz+rot6d
    # - hands: absolute finger joint targets
    "action": ModalityConfig(
        delta_indices=list(range(ACTION_HORIZON)),
        modality_keys=[
            "left_arm_joint",
            "right_arm_joint",
            "left_arm_ee_pose",
            "right_arm_ee_pose",
            "left_hand",
            "right_hand",
        ],
        action_configs=[
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="left_arm_joint",
            ),
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="right_arm_joint",
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="left_arm_ee_pose",
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="right_arm_ee_pose",
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="left_hand",
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="right_hand",
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["annotation.human.action.task_description"],
    ),
}


register_modality_config(marvin_dual_arm_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)
