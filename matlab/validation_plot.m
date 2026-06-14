function fig = validation_plot(run, out_path)
%VALIDATION_PLOT  Trajectory (East/Up) + speed/altitude vs time for a run.
%
%   FIG = VALIDATION_PLOT(RUN) plots, in a 1x3 layout:
%       (1) Vehicle vs target trajectory in the East-Up plane,
%       (2) Speed = |v| vs time,
%       (3) Altitude (Up) vs time,
%   reproducing the headline figures of the Python post-processing
%   (gncpost.plots.plot_trajectory + plot_states) from the SAME CSV data
%   contract (see docs/DATA_CONTRACT.md). RUN is the struct returned by
%   LOAD_RUN, or a path to a run folder (loaded for you).
%
%   VALIDATION_PLOT(RUN, OUT_PATH) also saves the figure to OUT_PATH (PNG).
%
%   Works in MATLAB and GNU Octave.
%
%   Example:
%       validation_plot('runs/sample_run', 'trajectory_matlab.png');
%
%   See also LOAD_RUN, ALLAN_DEVIATION.

    if ischar(run) || isstring(run)
        run = load_run(char(run));
    end
    if isempty(run.vehicle)
        error('validation_plot:noVehicle', 'run has no vehicle.csv data');
    end

    veh = run.vehicle;
    t   = getcol(veh, 't');
    vx  = getcol(veh, 'x');   % East
    vz  = getcol(veh, 'z');   % Up
    speed = sqrt(getcol(veh, 'vx').^2 + getcol(veh, 'vy').^2 + getcol(veh, 'vz').^2);

    has_target = ~isempty(run.target);
    if has_target
        tgt = run.target;
        tx = getcol(tgt, 'x');
        tz = getcol(tgt, 'z');
    end

    fig = figure('Name', 'gnc-sim validation', 'Color', 'w');

    % --- (1) Trajectory East-Up ---
    subplot(1, 3, 1);
    plot(vx, vz, '-', 'Color', [0.12 0.47 0.71], 'LineWidth', 1.8); hold on;
    plot(vx(1), vz(1), 'o', 'Color', [0.12 0.47 0.71], 'MarkerSize', 6);
    if has_target
        plot(tx, tz, '--', 'Color', [0.84 0.15 0.16], 'LineWidth', 1.5);
        plot(tx(end), tz(end), 'x', 'Color', [0.84 0.15 0.16], 'MarkerSize', 10, 'LineWidth', 2);
        legend('vehicle', 'launch', 'target', 'target end', 'Location', 'best');
    else
        legend('vehicle', 'launch', 'Location', 'best');
    end
    xlabel('East [m]'); ylabel('Up [m]');
    ttl = 'Vehicle vs Target — East-Up';
    if ~isempty(fieldnames(run.manifest))
        ttl = sprintf('%s\n(miss %.2f m, intercept=%d)', ttl, run.miss_distance, run.intercept);
    end
    title(ttl); grid on; axis equal;

    % --- (2) Speed vs time ---
    subplot(1, 3, 2);
    plot(t, speed, '-', 'Color', [0.17 0.63 0.17], 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('speed [m/s]'); title('Speed'); grid on;

    % --- (3) Altitude vs time ---
    subplot(1, 3, 3);
    plot(t, vz, '-', 'Color', [0.58 0.40 0.74], 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('altitude (Up) [m]'); title('Altitude'); grid on;

    if nargin >= 2 && ~isempty(out_path)
        % saveas works on both MATLAB and Octave; print gives finer DPI control on MATLAB.
        try
            print(fig, char(out_path), '-dpng', '-r130');
        catch
            saveas(fig, char(out_path));
        end
        fprintf('wrote %s\n', char(out_path));
    end
end
