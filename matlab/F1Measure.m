clear all;
root = 'C:\Users\Ankit\Documents\TAMU Succeed\Research\Autonomous Driving\segmented\';
predFiles = dir([root '\ICNet']);
trueFiles = dir([root '\truth']);
precision = [];
recall = [];
f1Measure = [];
accuracy = [];
for
i = 3
:
length(trueFiles)
pred = imread(

char (join([root

"ICNet\" predFiles(i).name], '')));
correct = imread(

char (join([root

"truth\" trueFiles(i).name], '')));
tp = numel(pred(pred == 255 & pred == correct));
fp = numel(pred(pred == 255 & pred
~= correct));
tn = numel(pred(pred == 0 & pred == correct));
fn = numel(pred(pred == 0 & pred
~= correct));
precision(i
-2) = tp / (tp + fp);
recall(i
-2) = tp / (tp + fn);
f1Measure(i
-2) = (2 .*
precision(i
-2) .*
recall(i
-2)) ./ (
precision(i
-2) +
recall(i
-2));
accuracy(i
-2) = (tp + tn) ./ (tp + tn + fn + fp);
end
        f1Measure = f1Measure(~isnan(f1Measure));
mean(f1Measure)