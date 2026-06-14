function [taus, adev, fig] = allan_deviation(x, dt, varargin)
%ALLAN_DEVIATION  Overlapping Allan deviation of a signal, with a log-log plot.
%
%   [TAUS, ADEV] = ALLAN_DEVIATION(X, DT) computes the overlapping Allan
%   deviation sigma(tau) of the uniformly-sampled rate signal X (sample period
%   DT seconds). This is the MATLAB twin of
%   sensors/allan_variance.py:overlapping_allan_deviation, using the identical
%   estimator (Riley, NIST SP 1065) so a MATLAB user recovers the same curve
%   from the same data.
%
%   [TAUS, ADEV, FIG] = ALLAN_DEVIATION(...) also draws the annotated log-log
%   figure: the measured sigma(tau) points plus the -1/2 (white noise) and +1/2
%   (rate random walk) slope guide lines and a marker at the curve minimum
%   (bias-instability region).
%
%   Name/value options:
%       'NumPoints'   (default 60)  number of log-spaced averaging factors
%       'MinClusters' (default 12)  drop tau with fewer non-overlapping clusters
%       'Plot'        (default true) set false to skip the figure
%       'Label'       (default 'signal') legend label / title suffix
%       'OutPath'     (default '')   if set, save the figure to this PNG
%
%   On the resulting plot:
%       slope -1/2  : white noise (ARW/VRW); N = sigma read at tau = 1 s
%       flat / min  : bias instability (best in-run stability)
%       slope +1/2  : rate random walk
%
%   Works in MATLAB and GNU Octave.
%
%   Example (characterize the in-sim measured accel from a run's sensors.csv):
%       run = load_run('runs/sample_run');
%       dt  = run.manifest.dt;
%       a   = getcol(run.sensors, 'imu_accel_meas_x');
%       allan_deviation(a, dt, 'Label', 'accel meas');
%
%   See also LOAD_RUN, VALIDATION_PLOT.

    p = parse_opts(varargin);

    x = x(:);
    n = numel(x);
    if n < 4
        error('allan_deviation:tooShort', 'need at least 4 samples for Allan variance');
    end

    % theta = integrated rate (phase). Overlapping AVAR of the rate x equals the
    % second difference of theta scaled by 1/(2 tau^2).
    theta = [0; cumsum(x)] * dt;   % length n+1

    % Averaging factors m (samples per bin), capped so each tau keeps at least
    % MinClusters non-overlapping clusters (floor(n/m) >= MinClusters).
    max_m = max(1, floor(n / max(1, p.MinClusters)));
    m_values = unique(floor(logspace(0, log10(max_m), p.NumPoints)));
    m_values = m_values(m_values >= 1);

    taus = m_values(:) * dt;
    adev = zeros(numel(m_values), 1);
    for k = 1:numel(m_values)
        m   = m_values(k);
        tau = m * dt;
        % diffs = theta[i+2m] - 2 theta[i+m] + theta[i]
        diffs = theta(2 * m + 1:end) - 2.0 * theta(m + 1:end - m) + theta(1:end - 2 * m);
        denom = 2.0 * tau * tau * numel(diffs);
        avar  = sum(diffs .^ 2) / denom;
        adev(k) = sqrt(avar);
    end

    fig = [];
    if p.Plot
        fig = plot_allan_curve(taus, adev, p.Label, p.OutPath);
    end
end


function fig = plot_allan_curve(taus, adev, label, out_path)
    fig = figure('Name', 'Allan deviation', 'Color', 'w');
    loglog(taus, adev, 'o', 'MarkerSize', 4, 'Color', [0.12 0.47 0.71], ...
        'DisplayName', sprintf('%s (measured)', label)); hold on;

    % White-noise -1/2 guide through tau = 1 s: read N as sigma at the tau
    % nearest 1 s scaled back onto the -1/2 line, then draw N/sqrt(tau).
    [~, i1] = min(abs(taus - 1.0));
    N = adev(i1) * sqrt(taus(i1));   % sigma(1s) on a pure -1/2 line
    guide_tau = [taus(1), taus(end)];
    loglog(guide_tau, N ./ sqrt(guide_tau), ':', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 1.2, 'DisplayName', 'slope -1/2 (white)');

    % Rate-random-walk +1/2 guide anchored at the longest-tau point.
    K = adev(end) * sqrt(3.0 / taus(end));
    loglog(guide_tau, K .* sqrt(guide_tau / 3.0), '-.', 'Color', [0.84 0.15 0.16], ...
        'LineWidth', 1.2, 'DisplayName', 'slope +1/2 (RRW)');

    % Bias-instability marker at the curve minimum.
    [bmin, imin] = min(adev);
    loglog(taus(imin), bmin, 's', 'Color', [0.0 0.0 0.0], 'MarkerSize', 9, ...
        'LineWidth', 1.4, 'DisplayName', sprintf('bias instab. ~ %.2e', bmin));

    xlabel('averaging time \tau [s]');
    ylabel('Allan deviation \sigma(\tau)');
    title(sprintf('Overlapping Allan deviation — %s', label));
    grid on;
    legend('Location', 'best');

    if ~isempty(out_path)
        try
            print(fig, char(out_path), '-dpng', '-r130');
        catch
            saveas(fig, char(out_path));
        end
        fprintf('wrote %s\n', char(out_path));
    end
end


function p = parse_opts(args)
    p = struct('NumPoints', 60, 'MinClusters', 12, 'Plot', true, ...
        'Label', 'signal', 'OutPath', '');
    for k = 1:2:numel(args)
        key = args{k};
        val = args{k + 1};
        if ~isfield(p, key)
            error('allan_deviation:badOption', 'unknown option "%s"', key);
        end
        p.(key) = val;
    end
end
