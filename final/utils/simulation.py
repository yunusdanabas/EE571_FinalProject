# Simulation utilities - built incrementally by part agents

import numpy as np


def simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts=None):
    """Simulate discrete-time linear system x[k+1] = Ad*x[k] + Bd*u[k], y[k] = Cd*x[k]."""
    n = Ad.shape[0]
    p = Cd.shape[0]
    
    x = np.zeros((n, N))
    y = np.zeros((p, N))
    
    x[:, 0] = x0
    y[:, 0] = Cd @ x[:, 0]
    
    for k in range(N - 1):
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
        y[:, k] = Cd @ x[:, k]
    
    y[:, N - 1] = Cd @ x[:, N - 1]
    
    if Ts is not None:
        t = np.arange(N) * Ts
    else:
        t = np.arange(N)
    
    return x, y, t


def simulate_observer(Ad, Bd, Cd, L, x0, xhat0, u, N, Ts=None):
    """Simulate plant-observer system: x[k+1] = Ad*x[k] + Bd*u[k], xhat[k+1] = Ad*xhat[k] + Bd*u[k] + L*(y[k] - yhat[k])."""
    n = Ad.shape[0]
    p = Cd.shape[0]
    
    x = np.zeros((n, N))
    xhat = np.zeros((n, N))
    y = np.zeros((p, N))
    yhat = np.zeros((p, N))
    e = np.zeros((n, N))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y[:, 0] = Cd @ x[:, 0]
    yhat[:, 0] = Cd @ xhat[:, 0]
    e[:, 0] = x[:, 0] - xhat[:, 0]
    
    for k in range(N - 1):
        y[:, k] = Cd @ x[:, k]
        yhat[:, k] = Cd @ xhat[:, k]
        y_error = y[:, k] - yhat[:, k]
        
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + L @ y_error
        e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
    
    y[:, N - 1] = Cd @ x[:, N - 1]
    yhat[:, N - 1] = Cd @ xhat[:, N - 1]
    
    if Ts is not None:
        t = np.arange(N) * Ts
    else:
        t = np.arange(N)
    
    return {'x': x, 'xhat': xhat, 'y': y, 'yhat': yhat, 'e': e, 't': t}


def simulate_lqr_with_observer(Ad, Bd, Cd, K, L, x0, xhat0, N, Ts):
    """Closed-loop simulation with observer-based LQR control: u[k] = -K @ xhat[k]."""
    n = Ad.shape[0]
    m = Bd.shape[1]
    p = Cd.shape[0]
    
    # Standard convention: x has length N+1, u has length N
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    u = np.zeros((m, N))
    y = np.zeros((p, N + 1))
    e = np.zeros((n, N + 1))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y[:, 0] = Cd @ x[:, 0]
    e[:, 0] = x[:, 0] - xhat[:, 0]
    
    for k in range(N):
        u[:, k] = -K @ xhat[:, k]
        y[:, k] = Cd @ x[:, k]
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
        
        yhat_k = Cd @ xhat[:, k]
        y_error = y[:, k] - yhat_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + L @ y_error
        e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
    
    y[:, N] = Cd @ x[:, N]
    t = np.arange(N + 1) * Ts
    
    return {'x': x, 'xhat': xhat, 'u': u, 'y': y, 'e': e, 't': t}


def simulate_lqr_reduced(Ad, Bd_red, Cd, K_red, L, x0, xhat0, N, Ts):
    """Closed-loop simulation with observer-based LQR control using reduced inputs: u_red[k] = -K_red @ xhat[k]."""
    n = Ad.shape[0]
    m_red = Bd_red.shape[1]
    p = Cd.shape[0]
    
    # Standard convention: x has length N+1, u_red has length N
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    u_red = np.zeros((m_red, N))
    y = np.zeros((p, N + 1))
    e = np.zeros((n, N + 1))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y[:, 0] = Cd @ x[:, 0]
    e[:, 0] = x[:, 0] - xhat[:, 0]
    
    for k in range(N):
        u_red[:, k] = -K_red @ xhat[:, k]
        y[:, k] = Cd @ x[:, k]
        x[:, k + 1] = Ad @ x[:, k] + Bd_red @ u_red[:, k]
        
        yhat_k = Cd @ xhat[:, k]
        y_error = y[:, k] - yhat_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd_red @ u_red[:, k] + L @ y_error
        e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
    
    y[:, N] = Cd @ x[:, N]
    t = np.arange(N + 1) * Ts
    
    return {'x': x, 'xhat': xhat, 'u_red': u_red, 'y': y, 'e': e, 't': t}


def simulate_kalman_noisy(Ad, Bd, Cd, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42):
    """Simulate stochastic system with Kalman filter (zero input)."""
    n = Ad.shape[0]
    m = Bd.shape[1]
    p = Cd.shape[0]
    
    np.random.seed(seed)
    
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    y_true = np.zeros((p, N + 1))
    y_meas = np.zeros((p, N + 1))
    yhat = np.zeros((p, N + 1))
    innovations = np.zeros((p, N))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y_true[:, 0] = Cd @ x[:, 0]
    
    for k in range(N):
        w_k = np.random.multivariate_normal(np.zeros(m), Qw)
        v_k = np.random.multivariate_normal(np.zeros(p), Rv)
        
        y_true[:, k] = Cd @ x[:, k]
        y_meas[:, k] = y_true[:, k] + v_k
        yhat[:, k] = Cd @ xhat[:, k]
        innovations[:, k] = y_meas[:, k] - yhat[:, k]
        
        x[:, k + 1] = Ad @ x[:, k] + Bd @ w_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Lk @ innovations[:, k]
    
    v_N = np.random.multivariate_normal(np.zeros(p), Rv)
    y_true[:, N] = Cd @ x[:, N]
    y_meas[:, N] = y_true[:, N] + v_N
    yhat[:, N] = Cd @ xhat[:, N]
    
    t = np.arange(N + 1) * Ts
    
    return {'x': x, 'xhat': xhat, 'y_true': y_true, 'y_meas': y_meas, 'yhat': yhat, 'innovations': innovations, 't': t}


def simulate_lqg(Ad, Bd, Cd, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42):
    """LQG closed-loop simulation: u[k] = -K @ xhat[k] with noisy measurements."""
    n = Ad.shape[0]
    m = Bd.shape[1]
    p = Cd.shape[0]
    
    np.random.seed(seed)
    
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    u = np.zeros((m, N))
    y_true = np.zeros((p, N + 1))
    y_meas = np.zeros((p, N + 1))
    yhat = np.zeros((p, N + 1))
    innovations = np.zeros((p, N))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y_true[:, 0] = Cd @ x[:, 0]
    v_0 = np.random.multivariate_normal(np.zeros(p), Rv)
    y_meas[:, 0] = y_true[:, 0] + v_0
    yhat[:, 0] = Cd @ xhat[:, 0]
    
    for k in range(N):
        u[:, k] = -K @ xhat[:, k]
        w_k = np.random.multivariate_normal(np.zeros(m), Qw)
        
        y_true[:, k] = Cd @ x[:, k]
        yhat[:, k] = Cd @ xhat[:, k]
        innovations[:, k] = y_meas[:, k] - yhat[:, k]
        
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k] + Bd @ w_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + Lk @ innovations[:, k]
        
        v_k = np.random.multivariate_normal(np.zeros(p), Rv)
        y_meas[:, k + 1] = Cd @ x[:, k + 1] + v_k
    
    y_true[:, N] = Cd @ x[:, N]
    yhat[:, N] = Cd @ xhat[:, N]
    t = np.arange(N + 1) * Ts
    
    return {'x': x, 'xhat': xhat, 'u': u, 'y_true': y_true, 'y_meas': y_meas, 'yhat': yhat, 'innovations': innovations, 't': t}


def simulate_lqg_augmented(Ad, Bd, Cmeas, Cy, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42):
    """LQG simulation with separate measurement and cost outputs: y_cost = Cy @ x, y_meas = Cmeas @ x + v."""
    n = Ad.shape[0]
    m = Bd.shape[1]
    p = Cmeas.shape[0]
    
    np.random.seed(seed)
    
    x = np.zeros((n, N + 1))
    xhat = np.zeros((n, N + 1))
    u = np.zeros((m, N))
    y_cost = np.zeros((2, N + 1))
    y_true = np.zeros((p, N + 1))
    y_meas = np.zeros((p, N + 1))
    yhat = np.zeros((p, N + 1))
    innovations = np.zeros((p, N))
    
    x[:, 0] = x0
    xhat[:, 0] = xhat0
    y_cost[:, 0] = Cy @ x[:, 0]
    y_true[:, 0] = Cmeas @ x[:, 0]
    v_0 = np.random.multivariate_normal(np.zeros(p), Rv)
    y_meas[:, 0] = y_true[:, 0] + v_0
    yhat[:, 0] = Cmeas @ xhat[:, 0]
    
    for k in range(N):
        u[:, k] = -K @ xhat[:, k]
        w_k = np.random.multivariate_normal(np.zeros(m), Qw)
        
        y_cost[:, k] = Cy @ x[:, k]
        y_true[:, k] = Cmeas @ x[:, k]
        yhat[:, k] = Cmeas @ xhat[:, k]
        innovations[:, k] = y_meas[:, k] - yhat[:, k]
        
        x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k] + Bd @ w_k
        xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + Lk @ innovations[:, k]
        
        v_k = np.random.multivariate_normal(np.zeros(p), Rv)
        y_meas[:, k + 1] = Cmeas @ x[:, k + 1] + v_k
    
    y_cost[:, N] = Cy @ x[:, N]
    y_true[:, N] = Cmeas @ x[:, N]
    yhat[:, N] = Cmeas @ xhat[:, N]
    t = np.arange(N + 1) * Ts
    
    return {'x': x, 'xhat': xhat, 'u': u, 'y_cost': y_cost, 'y_true': y_true, 'y_meas': y_meas, 'yhat': yhat, 'innovations': innovations, 't': t}
