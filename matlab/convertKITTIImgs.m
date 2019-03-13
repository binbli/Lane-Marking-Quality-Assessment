clear all;
root = 'C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\data_road\training\gt_image_2\';
files = dir(root);
for i = 1:length(files)
    if(contains(files(i).name, 'road'))
        im = imread(char(join([root files(i).name], '')));
        im(:,:,1) = 255 - im(:, :, 1);
        im(:,:,2) = zeros(size(im, 1), size(im, 2));
        im = im(:, :, 1) + im(:, :, 3);
        imwrite(im, ['C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\segmented\truth\segmented_' files(i).name]);
    end
end