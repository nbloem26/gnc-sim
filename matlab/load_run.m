function run = load_run(run_dir)
%LOAD_RUN  Load a gnc-sim run folder into a struct.
%
%   RUN = LOAD_RUN(RUN_DIR) reads the CSV telemetry + manifest written by the
%   native CLI (gncsim --out <dir>) and returns a struct mirroring the Python
%   gncpost.loaders.load_run() loader. It consumes the SAME language-neutral
%   data contract (see docs/DATA_CONTRACT.md §3-4): the C++ core, the Python
%   suite, and these MATLAB scripts all agree on the column names below.
%
%   A single run folder contains (all sharing the `t` column):
%       vehicle.csv  t,x,y,z,vx,vy,vz,roll,pitch,yaw,mass,mach,thrust
%       target.csv   t,x,y,z,vx,vy,vz
%       gnc.csv      t,accel_cmd_x..z,fin_x..z,los_angle,los_rate,v_closing,range,nav_x..z,nav_nis
%       sensors.csv  t,imu_accel_true_x,imu_accel_meas_x,imu_gyro_true_x,imu_gyro_meas_x,seeker_los_true,seeker_los_meas
%       track.csv    t,track_x..z,tgt_x..z,track_nis      (optional / zero by default)
%       discrim.csv  t,selected_obj,discrim_correct,discrim_margin  (optional / inert by default)
%       manifest.json
%   A Monte-Carlo batch additionally has summary.csv
%       (case,seed,miss_distance,intercept_time,intercept).
%
%   The returned struct has fields:
%       .path                 char, the run directory
%       .vehicle .target .gnc .sensors .track .discrim   tables (CSVs that exist)
%       .summary              table (only if summary.csv is present)
%       .manifest             struct decoded from manifest.json
%       .intercept            logical (from manifest)
%       .miss_distance        double  (from manifest)
%       .t                    column vector, the shared time base
%
%   Missing optional CSVs yield empty tables; a missing manifest yields an
%   empty struct. Works in both MATLAB and GNU Octave.
%
%   Example:
%       run = load_run('runs/sample_run');
%       fprintf('miss = %.3f m, intercept = %d\n', run.miss_distance, run.intercept);
%
%   See also VALIDATION_PLOT, ALLAN_DEVIATION.

    if nargin < 1 || isempty(run_dir)
        error('load_run:noDir', 'Usage: run = load_run(run_dir)');
    end
    if exist(run_dir, 'dir') ~= 7
        error('load_run:notFound', 'run folder not found: %s', run_dir);
    end

    run = struct();
    run.path = run_dir;

    % --- CSV telemetry (read whichever files exist) ---
    run.vehicle = read_csv_table(fullfile(run_dir, 'vehicle.csv'));
    run.target  = read_csv_table(fullfile(run_dir, 'target.csv'));
    run.gnc     = read_csv_table(fullfile(run_dir, 'gnc.csv'));
    run.sensors = read_csv_table(fullfile(run_dir, 'sensors.csv'));
    run.track   = read_csv_table(fullfile(run_dir, 'track.csv'));
    run.discrim = read_csv_table(fullfile(run_dir, 'discrim.csv'));

    summary_path = fullfile(run_dir, 'summary.csv');
    if exist(summary_path, 'file') == 2
        run.summary = read_csv_table(summary_path);
    end

    % --- manifest.json ---
    run.manifest = struct();
    manifest_path = fullfile(run_dir, 'manifest.json');
    if exist(manifest_path, 'file') == 2
        txt = fileread(manifest_path);
        run.manifest = jsondecode(txt);
    end

    % --- Convenience accessors mirroring the Python Run properties ---
    if isfield(run.manifest, 'intercept')
        run.intercept = logical(run.manifest.intercept);
    else
        run.intercept = false;
    end
    if isfield(run.manifest, 'miss_distance')
        run.miss_distance = double(run.manifest.miss_distance);
    else
        run.miss_distance = NaN;
    end
    if ~isempty(run.vehicle) && ismember('t', run.vehicle.Properties.VariableNames)
        run.t = run.vehicle.t;
    else
        run.t = [];
    end
end


function tbl = read_csv_table(path)
%READ_CSV_TABLE  Read a CSV with a header row into a table; empty if absent.
%   Uses readtable on MATLAB; falls back to a manual parser on Octave (which
%   lacks readtable) so the loader is portable.
    if exist(path, 'file') ~= 2
        tbl = table();
        return;
    end
    if exist('readtable', 'file') == 2 || exist('readtable', 'builtin') == 5
        tbl = readtable(path, 'Delimiter', ',', 'ReadVariableNames', true);
    else
        tbl = octave_read_csv(path);
    end
end


function tbl = octave_read_csv(path)
%OCTAVE_READ_CSV  Minimal CSV-with-header reader returning a struct of columns.
%   Returns a scalar struct whose fields are the column vectors (Octave has no
%   `table`; the rest of these scripts index columns by name via a small helper).
    fid = fopen(path, 'r');
    header = strsplit(strtrim(fgetl(fid)), ',');
    data = dlmread(path, ',', 1, 0);
    fclose(fid);
    tbl = struct();
    for k = 1:numel(header)
        tbl.(matlab.lang.makeValidName(header{k})) = data(:, k);
    end
end
