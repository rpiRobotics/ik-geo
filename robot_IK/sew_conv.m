classdef sew_conv
    properties
        V
    end
    
    methods
        function obj = sew_conv(V)
            obj.V = V;
        end
        
        function psi = fwd_kin(obj, S, E, W)
            p_SE = E - S;
            p_SW = W - S;
            w_hat = vec_normalize(p_SW);
            
            p = (eye(3) - w_hat*w_hat')*p_SE;
            
            psi = atan2( ...
                    w_hat'*cross(obj.V, p), ...
                    obj.V'*p);
        end

        function [k, n] = inv_kin(obj, S, W, psi)
            W_hat = vec_normalize(W-S);
            y_c = cross(W_hat, obj.V);
            y_c = y_c / norm(y_c);
            x_c = cross(y_c, W_hat);

            k = x_c * cos(psi) + y_c * sin(psi);
            n = cross(W_hat, k);
        end

        function [J_e, J_w] = jacobian(obj, S, E, W)
            E = E - S;
            W = W - S;
            w_hat = vec_normalize(W);
            
            ell = cross( cross(W, obj.V) , W);
            ell_hat = vec_normalize(ell);
            
            p = (eye(3) - w_hat*w_hat')*E;
            p_hat = vec_normalize(p);
            
            J_e = cross(w_hat, p_hat)' / norm(p);
            
            J_w_1 = (obj.V'*W) /norm(ell) * cross(w_hat, ell_hat)';
            J_w_2 = (w_hat'*E)/norm(W)/norm(p) *  cross(w_hat, p_hat)';
            
            J_w = J_w_1 - J_w_2;
        end
    end
end

function n = vec_normalize(vec)
    n =  vec / norm(vec);
end