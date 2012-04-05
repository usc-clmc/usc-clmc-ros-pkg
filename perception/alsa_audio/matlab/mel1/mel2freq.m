function b = mel2freq (m)
% compute frequency from mel value
b = 700*((10.^(m ./2595)) -1);