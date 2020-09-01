#!/usr/bin/env python3


class Config:

	Project_Name = 'manifold_move'

	Num_Sick_Input = 540*2
	Num_Obj_Input = 2
	Num_Agentposi_Input = 4
	Num_Policy_Output = 4
	Num_Value_Output = 1

	Device = 'gpu:0'

	Learning_Rate_Start = 0.0003
	Learning_Rate_End = 0.0003
	Log_Epsilon = 1e-6

	LOAD_CHECKPOINT = True
	TENSORBOARD = False
	SAVE_MODELS = True
	SAVE_FREQUENCY = 1000

	DUAL_RMSPROP = True
	USE_GRAD_CLIP = True

	RMSPROP_DECAY = 0.99
	RMSPROP_MOMENTUM = 0.0
	RMSPROP_EPSILON = 0.1

	GRAD_CLIP_NORM = 40.0 
	MAX_GLOBAL_EP = 1000
	Num_STEPS = 100
	MAX_EP_STEP = 100