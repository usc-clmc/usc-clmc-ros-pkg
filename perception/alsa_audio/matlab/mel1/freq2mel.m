function m = freq2mel (f)
% compute mel value from frequency f
m = 2595 * log10(1 + f./700);
