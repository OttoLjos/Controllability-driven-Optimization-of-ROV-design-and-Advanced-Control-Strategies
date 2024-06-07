function v_dot = calcAcceleration(M, C, D, G)
    syms u v w p q r x y z phi theta pzi real
    Tau = sym('tau', [1 6]);
    assume(Tau, 'real')
    v_dot = M\(-C*[u, v, w, p, q, r]'-D*[u, v, w, p, q, r]' - G + Tau');
    open_system('modelsimulator')
    
    matlabFunctionBlock('modelsimulator/ROV/ROVSubsystem/ROV', v_dot(1), v_dot(2), v_dot(3), v_dot(4), v_dot(5), v_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi Tau], ...
        'Outputs', {'u_dot','v_dot','w_dot','p_dot','q_dot','r_dot',})

    matlabFunctionBlock('modelsimulator/ROV1/ROVSubsystem/ROV', v_dot(1), v_dot(2), v_dot(3), v_dot(4), v_dot(5), v_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi Tau], ...
        'Outputs', {'u_dot','v_dot','w_dot','p_dot','q_dot','r_dot',})

    matlabFunctionBlock('modelsimulator/ROV2/ROVSubsystem/ROV', v_dot(1), v_dot(2), v_dot(3), v_dot(4), v_dot(5), v_dot(6), ...
        'vars', [u v w p q r x y z phi theta pzi Tau], ...
        'Outputs', {'u_dot','v_dot','w_dot','p_dot','q_dot','r_dot',})
    
end

