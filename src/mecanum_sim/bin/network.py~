#!/usr/bin/env python3
import os
import re
import numpy as np
import tensorflow as tf 
import tensorflow_probability as tfp 
import time

from env import RLMoMaBot
import rospy
import sys
from Config import Config

CONV1_SIZE = 5
CONV1_DEEP = 32

CONV2_SIZE = 3
CONV2_DEEP = 32
GAMMA = 0.90                # discount factor

FC_SIZE = 32
ENTROPY_BETA = 0.01

LR_A = 0.0001               # learning rate for actor
LR_C = 0.001                # learning rate for critic

MAX_GLOBAL_EP = Config.MAX_GLOBAL_EP
Num_STEPS = Config.Num_STEPS
MAX_EP_STEP = Config.MAX_EP_STEP

class network:
	def __init__(self):
		sys.path.append('/home/jeffrey/catkin_ws/src/mecanum_sim/src')
		self.device = Config.Device
		self.num_actions = Config.Num_Policy_Output
		
		self.num_sick_input = Config.Num_Sick_Input
		self.num_obj_input = Config.Num_Obj_Input
		self.num_agentposi_input = Config.Num_Agentposi_Input
		self.num_value = 1
		self.num_actions = 4

		self.learning_rate = Config.Learning_Rate_Start
		self.log_epsilon = Config.Log_Epsilon
		self.graph = tf.Graph()
		with self.graph.as_default() as g:
			with tf.device(self.device):
				self._create_graph()

				self.sess = tf.Session(
					graph=self.graph,
					config=tf.ConfigProto(
						allow_soft_placement=True,
						log_device_placement=False))
						##gpu_option=tf.GPUOptions(allow_growth=True)))
				self.sess.run(tf.global_variables_initializer())

				if Config.TENSORBOARD: self._create_tensorboard()
				if Config.LOAD_CHECKPOINT or Config.SAVE_MODELS:
					vars = tf.global_variables()
					self.saver = tf.train.Saver({var.name: var for var in vars}, max_to_keep=0)


	def _create_graph(self):
		self.x = tf.placeholder(tf.float32,[None, 1, self.num_sick_input, 1], name='X')
		self.obj = tf.placeholder(tf.float32, [None, self.num_obj_input], name='Object')
		self.posi = tf.placeholder(tf.float32, [None, self.num_agentposi_input], name='Position')
		self.v_target = tf.placeholder(tf.float32, [None, self.num_value], name='Vtarget') # v_target value
		self.a_his = tf.placeholder(tf.float32, [None, self.num_actions], name='A') 

		self.var_learning_rate = tf.placeholder(tf.float32, [], name='lr')
		self.global_step = tf.Variable(0, trainable=False, name='step')

		with tf.variable_scope('layer1-conv1'):
			self.conv1_weights = tf.get_variable("weight", [1, 4, 1, 1], initializer = tf.truncated_normal_initializer(stddev = 0.1))
			print(self.conv1_weights)
			self.conv1_biases = tf.get_variable("bias", [1], initializer = tf.constant_initializer(0.0))
			print(self.conv1_biases)
			self.conv1 = tf.nn.conv2d(self.x, self.conv1_weights, strides = [1,1,4,1], padding = 'VALID')
			print(self.conv1)
			self.relu1 = tf.nn.relu(tf.nn.bias_add(self.conv1, self.conv1_biases))
			print(self.relu1)

		with tf.name_scope('layer2-pool1'):
			self.pool1 = tf.nn.max_pool(self.relu1, ksize = [1,1,2,1], strides = [1,1,2,1], padding = 'SAME')
			print(self.pool1)

		with tf.variable_scope('layer3-conv2'):
			self.conv2_weights = tf.get_variable("weight", [1, 4, 1, 1], initializer = tf.truncated_normal_initializer(stddev = 0.1))
			print(self.conv2_weights)
			self.conv2_biases = tf.get_variable("bias", [1], initializer = tf.constant_initializer(0.0))
			print(self.conv2_biases)
			self.conv2 = tf.nn.conv2d(self.pool1, self.conv2_weights, strides=[1,1,1,1], padding = 'SAME')
			print(self.conv2)
			self.relu2 = tf.nn.relu(tf.nn.bias_add(self.conv2, self.conv2_biases))
			print(self.relu2)

		with tf.name_scope('layer4-pool2'):
			self.pool2 = tf.nn.max_pool(self.relu2, ksize = [1,1,2,1], strides = [1,1,2,1], padding = 'SAME')
			print(self.pool2)

		pool_shape = self.pool2.get_shape().as_list()
		nodes = pool_shape[1]*pool_shape[2]*pool_shape[3]
		print(nodes)

		self.reshaped = tf.reshape(self.pool2,[-1, nodes])
		print(self.reshaped)

		with tf.variable_scope('layer5-fc1'):
			self.fc1_weights = tf.get_variable("weight", [nodes, FC_SIZE], initializer = tf.truncated_normal_initializer(stddev = 0.1))
			self.fc1_biases = tf.get_variable("bias", [FC_SIZE], initializer = tf.constant_initializer(0.1))
			self.fc1 = tf.nn.relu(tf.matmul(self.reshaped, self.fc1_weights)+self.fc1_biases)
			print(self.fc1)

		self.layer_all_input = tf.concat([self.fc1, self.obj, self.obj, self.obj, self.obj, self.obj, self.posi, self.posi, self.posi, self.posi, self.posi], 1, name = 'all-input')
		print(self.layer_all_input)
		self.fc_all_input1 = tf.layers.dense(inputs = self.layer_all_input, units = FC_SIZE, activation = tf.nn.relu, name = 'fc-all-input1')
		print(self.fc_all_input1)
		self.fc_all_input2 = tf.layers.dense(inputs = self.fc_all_input1, units = FC_SIZE, activation = tf.nn.relu, name = 'fc-all-input2')
		print(self.fc_all_input2)

		w_init = tf.random_normal_initializer(0., .1)
		self.value = tf.layers.dense(inputs = self.fc_all_input2, units = 1, activation = tf.nn.relu, kernel_initializer=w_init, name = 'value-out')
		print(self.value)
		w_init_p = tf.random_normal_initializer(0., .1)
		self.policy_mu = tf.layers.dense(inputs = self.fc_all_input2, units = self.num_actions, activation = tf.nn.tanh, kernel_initializer=w_init_p, name = 'policy-out')
		print(self.policy_mu)
		self.policy_sigma = tf.layers.dense(inputs = self.fc_all_input2, units = self.num_actions, activation = tf.nn.softplus, kernel_initializer=w_init_p, name='sigma') # estimated variance
		print(self.policy_sigma)

		A_BOUND=[-0.1,0.1]
		tfd = tfp.distributions
		normal_dist = tfd.Normal(loc=A_BOUND[1]*self.policy_mu, scale=0.001*self.policy_sigma)
		print(normal_dist)
		td = tf.subtract(self.v_target, self.value, name='TD_error')

		with tf.name_scope('a_loss'):
			log_prob = normal_dist.log_prob(self.a_his)
			exp_v = log_prob * td
			entropy = normal_dist.entropy() 
			self.exp_v = ENTROPY_BETA * entropy + exp_v
			self.a_loss = tf.reduce_mean(-self.exp_v)
		

		with tf.name_scope('choose_a'):  # use local params to choose action
			self.A = tf.clip_by_value(tf.squeeze(normal_dist.sample(1), axis=0), A_BOUND[0], A_BOUND[1]) # sample a action from distribution
			print(self.A )

		with tf.name_scope('v_loss'):
			self.v_loss = tf.reduce_mean(tf.square(td))
			print(self.v_loss)

		if Config.DUAL_RMSPROP:
			self.opt_p = tf.train.RMSPropOptimizer(LR_A, name='RMSPropA')##tf.train.RMSPropOptimizer(learning_rate = self.var_learning_rate,decay = Config.RMSPROP_DECAY,momentum = Config.RMSPROP_MOMENTUM,epsilon = Config.RMSPROP_EPSILON)
			self.opt_v = tf.train.RMSPropOptimizer(LR_C, name='RMSPropC')##tf.train.RMSPropOptimizer(learning_rate = self.var_learning_rate,decay = Config.RMSPROP_DECAY,momentum = Config.RMSPROP_MOMENTUM,epsilon = Config.RMSPROP_EPSILON)

		else:
			self.all_loss = self.a_loss + self.v_loss
			self.opt = tf.train.RMSPropOptimizer(LR_C, name='RMSPropC')##tf.train.RMSPropOptimizer(learning_rate = self.var_learning_rate,decay = Config.RMSPROP_DECAY,momentum = Config.RMSPROP_MOMENTUM,epsilon = Config.RMSPROP_EPSILON)

		if Config.USE_GRAD_CLIP:
			if Config.DUAL_RMSPROP:
				self.opt_grad_v = self.opt_v.compute_gradients(self.v_loss)
				self.opt_grad_v_clipped = [(tf.clip_by_norm(g, Config.GRAD_CLIP_NORM), v) for g, v in self.opt_grad_v if not g is None]
				self.train_op_v = self.opt_v.apply_gradients(self.opt_grad_v_clipped)

				self.opt_grad_p = self.opt_p.compute_gradients(self.a_loss)
				self.opt_grad_p_clipped = [(tf.clip_by_norm(g, Config.GRAD_CLIP_NORM), v) for g, v in self.opt_grad_p if not g is None]
				self.train_op_p = self.opt_p.apply_gradients(self.opt_grad_p_clipped)
				self.train_op = [self.train_op_p, self.train_op_v]
			else:
				self.opt_grad = self.opt.compute_gradients(self.all_loss)
				self.opt_grad_clipped = [(tf.clip_by_average_norm(g, Config.GRAD_CLIP_NORM),v) for g,v in self.opt_grad]
				self.train_op = self.opt.apply_gradients(self.opt_grad_clipped)

		else:
			if Config.DUAL_RMSPROP:
				self.train_op_v = self.opt_p.minimize(self.v_loss, global_step=self.global_step)
				self.train_op_p = self.opt_v.minimize(self.a_loss, global_step=self.global_step)
				self.train_op = [self.train_op_p, self.train_op_v]
			else:
				self.train_op = self.opt.minimize(self.all_loss, global_step=self.global_step)

		self.saver = tf.train.Saver()



	def predict_p_mu(self, s, o, p):
		s_reshaped = np.reshape(s,[1,1,self.num_sick_input,1])
		o_reshaped = np.expand_dims(o,axis=0)##np.reshape(o,[1,self.num_obj_input])
		p_reshaped = np.reshape(p,[1,self.num_agentposi_input])
		return self.sess.run(self.policy_mu, feed_dict = {self.x: s_reshaped, self.obj: o_reshaped, self.posi: p_reshaped})

	def predict_p_sigma(self, s, o, p):
		s_reshaped = np.reshape(s,[1,1,self.num_sick_input,1])
		o_reshaped = np.expand_dims(o,axis=0)##np.reshape(o,[1,self.num_obj_input])
		p_reshaped = np.reshape(p,[1,self.num_agentposi_input])
		return self.sess.run(self.policy_sigma, feed_dict = {self.x: s_reshaped, self.obj: o_reshaped, self.posi: p_reshaped})

	def predict_v(self, s, o, p):
		s_reshaped = np.reshape(s,[1,1,self.num_sick_input,1])
		o_reshaped = np.expand_dims(o,axis=0)##np.reshape(o,[1,self.num_obj_input])
		p_reshaped = np.reshape(p,[1,self.num_agentposi_input])
		return self.sess.run(self.value, feed_dict = {self.x: s_reshaped, self.obj: o_reshaped, self.posi: p_reshaped})


	def simple_load(self, filename=None):
		if filename is None:
			print ("[network.py] Didn't define simple_load filename")
        ##ckpt = tf.train.get_checkpoint_state(filename+'/network_02360000')
        ##self.saver = tf.train.import_meta_graph(filename+'network_01900000'+'.meta')
        ##self.saver.restore(self.sess, os.path.join(filename,'network_02360000'))
       
		config = tf.ConfigProto(allow_soft_placement = True)
		with tf.Session(config = config) as sess:
			sess.run(tf.global_variables_initializer())
        #self.saver.restore(self.sess, module_file)
		self.saver.restore(self.sess, os.path.join(filename))

	def update_global(self, feed_dict):  # run by a local
		self.sess.run([self.train_op_p, self.train_op_v], feed_dict)  # local grads applies to global net

	def choose_action(self, s, o, p):  # run by a local
		##s_reshaped = tf.reshape(s,[1, 1368, 1])
		##print("choose_action s :",s)
		##print("choose_action o :",o)
		##print("choose_action p :",p)
		s_reshaped = np.reshape(s,[1,1,self.num_sick_input,1])
		o_reshaped = np.expand_dims(o,axis=0)##np.reshape(o,[1,self.num_obj_input])
		p_reshaped = np.reshape(p,[1,self.num_agentposi_input])
		return self.sess.run(self.A, {self.x: s_reshaped, self.obj: o_reshaped, self.posi: p_reshaped})[0]


    
	def train(self):
		print("Begin train now :")
		self.env = RLMoMaBot()
		self.steps = 0
		global_episodes=0
		total_step = 0
		for global_episodes in range(MAX_GLOBAL_EP):
			print("=========Begin global_episodes :=========",global_episodes)
			checkpoint_folder='/home/jeffrey/catkin_ws/saved_networks/'
			path = tf.train.latest_checkpoint(checkpoint_folder)
			if path is None:
				print('Initializing all variables')
				##self.sess.run(tf.global_variables_initializer())
			else:
				print('Restoring network variables from previous run')
				self.saver.restore(self.sess, path)
				last_saving_step = int(path[path.rindex('-')+1:])
				global_episodes = last_saving_step+1
				print("Global episodes is :",global_episodes)
			self.env.reset()
			ep_r = 0
			for total_step in range(Num_STEPS):
				print("---------Begin total_step :---------",total_step)
				if total_step == 0:
					a = [0,0,0,0]
					#mu = [0,0,0,0]
					#sigma = [0,0,0,0]
				else:
					a = self.choose_action(s, o, p)         # estimate stochastic action based on policy 
					#mu = self.predict_p_mu(s, o, p)
					#sigma = self.predict_p_sigma(s, o, p)
				print("choose action :",a)
				#print("mu :",mu)
				#print("sigma :",sigma)
				s_, o_, p_, r = self.env.step(a) # make step in environment
				self.env.Num_step=self.env.Num_step+1
				self.steps = self.steps+1
				if self.env.Num_step>=200:
					self.env.Num_step=self.env.Num_step-200
				s_reshaped = np.reshape(s_,[1,1,self.num_sick_input,1])
				o_reshaped = np.reshape(o_,[1,self.num_obj_input])##np.expand_dims(o_,axis=0)
				p_reshaped = np.reshape(p_,[1,self.num_agentposi_input])
				v_s_ = self.sess.run(self.value, {self.x: s_reshaped, self.obj: o_reshaped, self.posi: p_reshaped})[0]
            	##print("v_s_:",v_s_)
				v_s_ = r + GAMMA * v_s_
				v_s_reshaped = np.reshape(v_s_,[1,self.num_value])
				a_reshaped = np.reshape(a,[1,self.num_actions])
				feed_dict = {self.x: s_reshaped,self.obj: o_reshaped,self.posi: p_reshaped,self.a_his: a_reshaped,self.v_target: v_s_reshaped}
				loss = self.update_global(feed_dict) # actual training step, update global ACNet
				s = s_
				o = o_ 
				p = p_
				print("Episode", global_episodes, "Step", total_step, "Object", o,"Position", p,"Action", a, "Reward", r, "Loss", loss)
				if self.steps % MAX_EP_STEP == 0:
					self.saver.save(self.sess, 'saved_networks/' + 'network' + '-ca3c', global_step = global_episodes)

            

if __name__ == '__main__':

	net = network()
	net.train()




