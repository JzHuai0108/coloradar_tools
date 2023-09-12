function tform = read_transform(fn)
    did = fopen(fn, 'r');
    c = textscan(did, '%f');
    fclose(did);
    W0_T_W = reshape(c{1}, 4, 3)';
    tform = [W0_T_W; 0, 0, 0, 1];
    % tform = [transpose(W0_T_W(1:3, 1:3)), W0_T_W(1:3, 4); 0, 0, 0, 1];
end
