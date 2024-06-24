import numpy as np
import pbdlib as pbd
from sklearn.base import BaseEstimator

class HMMLQR(BaseEstimator):
    def __init__(self, nb_states=5, reg1=1e-3, reg2=1e-3, reg3=1e-3, reg4=1e-3, 
                 reg5=1e-3, reg6=1e-3, reg7=1e-3, reg8=1e-3, reg9=1e-3, 
                 reg10=1e-3, reg11=1e-3, reg12=1e-3, gmm_u=1.0):
        self.nb_states = nb_states
        self.reg1 = reg1
        self.reg2 = reg2
        self.reg3 = reg3
        self.reg4 = reg4
        self.reg5 = reg5
        self.reg6 = reg6
        self.reg7 = reg7
        self.reg8 = reg8
        self.reg9 = reg9
        self.reg10 = reg10
        self.reg11 = reg11
        self.reg12 = reg12
        self.gmm_u = gmm_u
        return

    def fit(self, data):
        data_start = []
        data_end = []
#        reg = [self.reg1, self.reg2, self.reg3, self.reg4, self.reg5, self.reg6,
#               self.reg7, self.reg8, self.reg9, self.reg10,self.reg11,self.reg12]
        reg = 1e-8 
        
        for p in data:
            # Set start point
            start = p[0, :]
            arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, start)
            data_start.append(arr_trans)

        for p in data:
            # Set end point
            end = p[-1, :]
            arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, end)
            data_end.append(arr_trans)

        hmm_demos = [np.hstack([s, e]) for e in data_end for s in data_start]
        
        self.hmm_ = pbd.HMM(nb_dim=12, nb_states=self.nb_states)
        
        self.hmm_.init_hmm_kbins(hmm_demos)
        
        self.hmm_.em(hmm_demos, reg=reg)

        sqs = []
        for hmm_demo in hmm_demos:
            sqs.append(self.hmm_.viterbi(hmm_demo))

        arr_sqs = np.asarray(sqs)
        opt_sq = []
        for col in arr_sqs.T:
            opt_sq.append(np.bincount(col).argmax())

        self.sq = opt_sq

        return self

    def predict(self, data, hor=None):
        try:
            getattr(self, "hmm_")
        except AttributeError:
            raise RuntimeError("You must train classifer before predicting data!")

        if not hor:
            hor = data[0].shape[0]+1

        A, b = pbd.utils.get_canonical(6, 1, 0.025)
        repros = []

        for i in range(len(data)):
            A0, b0 = pbd.utils.inv_for_lintrans(data[i][0,:])
            An, bn = pbd.utils.inv_for_lintrans(data[i][-1,:])
            
            dim1 = slice(0,6)
            dim2 = slice(6,12)
            
            _mod1 = self.hmm_.marginal_model(dims = dim1).lintrans(A0, b0)
            _mod2 = self.hmm_.marginal_model(dims = dim2).lintrans(An, bn)
            
            # product 
            _prod = _mod1 * _mod2
            
            # get the most probable sequence of state for this demonstration
            data_start = []
            data_end = []
            
            for p in data:
                # Set start point
                start = p[0, :]
                arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, start)
                data_start.append(arr_trans)

            for p in data:
                # Set end point
                end = p[-1, :]
                arr_trans = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, p, end)
                data_end.append(arr_trans)

            # solving LQR
            lqr = pbd.LQR(A, b, dt=0.025, horizon=hor)
            lqr.gmm_xi = _prod, self.sq
            lqr.gmm_u = self.gmm_u
            lqr.ricatti()
            
            pos = data[i][0]
            xi, _ = lqr.get_seq(pos)
            
            repros.append(xi)

        return repros

    def score(self, demo):
        costsum = 0

        repro = self.predict(demo)
        for i in range(len(repro)):
            ## Cost function
            # Initial pose
            diff_init = sum(((repro[i][0, :] - demo[i][0])*np.array([10,5,20,20,20,10]))**2)

            # Final pose
            diff_end = sum(((repro[i][-1, :] - demo[i][-1])*np.array([10,5,20,20,20,10]))**2)

            # Trajectory length
            diff_repro = np.diff(repro[i], axis=0)
            path_length = sum(np.apply_along_axis(pbd.utils.pythagoras_3d, 1, diff_repro))*6

            # Initial trajectory
            start_repro = np.apply_along_axis(pbd.utils.transform_matrix_3D, 1, repro[i], demo[i][0])
            row_index = np.argmax(start_repro[:, 2] > 0.025)
            diff_traj = (sum(((start_repro[row_index, [0, 1, 3, 4]])*np.array([10,5,20,20]))**2))*1.5

            # Sum costs
            costsum -= diff_init
            costsum -= diff_end
            costsum -= path_length
            costsum -= diff_traj

        return costsum / len(demo)
