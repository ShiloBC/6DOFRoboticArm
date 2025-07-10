function f = create_s_func(d, T_val)
    syms t t1 t2 T a;
    assume(0 < t1 & t1 < t2 & t2 < T);
    
    t1_val = d;
    t2_val = T_val - d;
    

    s_dot_dot = piecewise(...
        0 <= t & t < t1, a, ...
        t1 <= t & t < t2, 0, ...
        t2 <= t & t <= T, -a);
    s_dot = int(s_dot_dot, t, 0, t);
    s = int(s_dot, t, 0, t);
    s = simplify(s);
    s = subs(s, [t1 t2 T], [t1_val t2_val T_val]);

    
    a_val = double(a/subs(s, t, T_val));
    s = subs(s, a, a_val);

    
    matlabFunction(s, 'Vars', t, 'File', 'get_s');
    f = @get_s;
end
