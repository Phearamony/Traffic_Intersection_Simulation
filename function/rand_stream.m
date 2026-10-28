function r = rand_stream(seed, m, n, kind)
%RAND_STREAM  Draw m-by-n numbers from an independent seeded stream.
%   r = rand_stream(seed, m, n)          uniform (0,1)
%   r = rand_stream(seed, m, n, 'randn') standard normal
%   The global random state is not touched. MATLAB uses RandStream
%   (Mersenne twister); Octave, which has no RandStream, saves and restores
%   the global generator. The two give different numbers for the same seed,
%   which only matters when comparing a MATLAB run with an Octave test.
if nargin < 4, kind = 'rand'; end
if exist('RandStream', 'class') == 8 || exist('RandStream') == 2 %#ok<EXIST>
    rs = RandStream('mt19937ar', 'Seed', seed);
    if strcmp(kind, 'randn'), r = randn(rs, m, n); else, r = rand(rs, m, n); end
else
    su = rand('state'); sn = randn('state'); %#ok<RAND>
    rand('twister', seed); randn('state', seed); %#ok<RAND>
    if strcmp(kind, 'randn'), r = randn(m, n); else, r = rand(m, n); end
    rand('state', su); randn('state', sn); %#ok<RAND>
end
end
