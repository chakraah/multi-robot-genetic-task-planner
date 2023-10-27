function gridMap = binarize_aerial_image(imageOriginalPath, imageMaskPath, gridSize, threshold)
% Binarization of an aerial image
% Inputs:
%   imageOriginalPath: Path to the original aerial image
%   imageMaskPath: Path to the image mask
%   gridSize: Size of the binary matrix representing the environment with obstacles
%   threshold: Threshold value for converting the grayscale mask to binary
% Outputs:
%   gridMap: Binary matrix representing the environment

tic; % start timer

% Load the aerial view image and the mask into MATLAB
imgMask = imread(imageMaskPath);
imgOriginal = imread(imageOriginalPath);

% Convert the image mask to grayscale
grayMask = rgb2gray(imgMask);

% Resize the grayscale mask to the desired size for the grid map
resizedGrayMask = imresize(grayMask, gridSize);

% Convert the grayscale mask to a binary image using the specified threshold
gridMap = im2bw(resizedGrayMask, threshold);

 % Define the number of rows and columns for the grid
numRows = gridSize(1);
numCols = gridSize(2);

% Get dimensions of the original image
[height, width, ~] = size(imgOriginal);

% Calculate grid spacing
rowSpacing = floor(height / numRows);
colSpacing = floor(width / numCols);

% Create a figure and display the original image
nexttile;
imshow(imgOriginal);
hold on;

% Draw horizontal grid lines on the original image
for i = 1:numRows
    y = i * rowSpacing;
    line([1, width], [y, y], 'Color', 'k', 'LineWidth', 1);  % Red lines for the grid
end

% Draw vertical grid lines on the original image
for j = 1:numCols
    x = j * colSpacing;
    line([x, x], [1, height], 'Color', 'k', 'LineWidth', 1);  % Red lines for the grid
end

title('Original Image with Grid Lines');

% Display the binary image with grid lines
nexttile;
imshow(gridMap)
hold on;
ax = gca;

% Add grid lines manually to the binary image
hold on;

% Draw vertical grid lines
for col = 1:numCols
    x = col - 0.5;
    plot([x, x], [0.5, numRows + 0.5], 'color', [0 0 0], 'LineWidth', 1.5, 'LineStyle', '-');
end

% Draw horizontal grid lines
for row = 1:numRows
    y = row - 0.5;
    plot([0.5, numCols + 0.5], [y, y], 'color', [0 0 0], 'LineWidth', 1.5, 'LineStyle', '-');
end

title('Binary Image with Grid Lines');
axis equal;

% Stop measuring execution time
elapsed_time = toc;

% Display the execution time
fprintf('Binarization took %.4f seconds.\n', elapsed_time);

end
