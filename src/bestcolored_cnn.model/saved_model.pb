??
??
B
AssignVariableOp
resource
value"dtype"
dtypetype?
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
?
Conv2D

input"T
filter"T
output"T"
Ttype:	
2"
strides	list(int)"
use_cudnn_on_gpubool(",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 "-
data_formatstringNHWC:
NHWCNCHW" 
	dilations	list(int)

.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
?
MaxPool

input"T
output"T"
Ttype0:
2	"
ksize	list(int)(0"
strides	list(int)(0",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 ":
data_formatstringNHWC:
NHWCNCHWNCHW_VECT_C
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(?

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype?
E
Relu
features"T
activations"T"
Ttype:
2	
[
Reshape
tensor"T
shape"Tshape
output"T"	
Ttype"
Tshapetype0:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
0
Sigmoid
x"T
y"T"
Ttype:

2
?
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring ?
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
?
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 ?"serve*2.4.12v2.4.0-49-g85c8b2a817f8??
?
conv2d_24/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape: *!
shared_nameconv2d_24/kernel
}
$conv2d_24/kernel/Read/ReadVariableOpReadVariableOpconv2d_24/kernel*&
_output_shapes
: *
dtype0
t
conv2d_24/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameconv2d_24/bias
m
"conv2d_24/bias/Read/ReadVariableOpReadVariableOpconv2d_24/bias*
_output_shapes
: *
dtype0
?
conv2d_25/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:  *!
shared_nameconv2d_25/kernel
}
$conv2d_25/kernel/Read/ReadVariableOpReadVariableOpconv2d_25/kernel*&
_output_shapes
:  *
dtype0
t
conv2d_25/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameconv2d_25/bias
m
"conv2d_25/bias/Read/ReadVariableOpReadVariableOpconv2d_25/bias*
_output_shapes
: *
dtype0
?
conv2d_26/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape: @*!
shared_nameconv2d_26/kernel
}
$conv2d_26/kernel/Read/ReadVariableOpReadVariableOpconv2d_26/kernel*&
_output_shapes
: @*
dtype0
t
conv2d_26/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*
shared_nameconv2d_26/bias
m
"conv2d_26/bias/Read/ReadVariableOpReadVariableOpconv2d_26/bias*
_output_shapes
:@*
dtype0
{
dense_15/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?2* 
shared_namedense_15/kernel
t
#dense_15/kernel/Read/ReadVariableOpReadVariableOpdense_15/kernel*
_output_shapes
:	?2*
dtype0
r
dense_15/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_namedense_15/bias
k
!dense_15/bias/Read/ReadVariableOpReadVariableOpdense_15/bias*
_output_shapes
:*
dtype0
f
	Adam/iterVarHandleOp*
_output_shapes
: *
dtype0	*
shape: *
shared_name	Adam/iter
_
Adam/iter/Read/ReadVariableOpReadVariableOp	Adam/iter*
_output_shapes
: *
dtype0	
j
Adam/beta_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_1
c
Adam/beta_1/Read/ReadVariableOpReadVariableOpAdam/beta_1*
_output_shapes
: *
dtype0
j
Adam/beta_2VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_2
c
Adam/beta_2/Read/ReadVariableOpReadVariableOpAdam/beta_2*
_output_shapes
: *
dtype0
h

Adam/decayVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name
Adam/decay
a
Adam/decay/Read/ReadVariableOpReadVariableOp
Adam/decay*
_output_shapes
: *
dtype0
x
Adam/learning_rateVarHandleOp*
_output_shapes
: *
dtype0*
shape: *#
shared_nameAdam/learning_rate
q
&Adam/learning_rate/Read/ReadVariableOpReadVariableOpAdam/learning_rate*
_output_shapes
: *
dtype0
^
totalVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nametotal
W
total/Read/ReadVariableOpReadVariableOptotal*
_output_shapes
: *
dtype0
^
countVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_namecount
W
count/Read/ReadVariableOpReadVariableOpcount*
_output_shapes
: *
dtype0
b
total_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	total_1
[
total_1/Read/ReadVariableOpReadVariableOptotal_1*
_output_shapes
: *
dtype0
b
count_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	count_1
[
count_1/Read/ReadVariableOpReadVariableOpcount_1*
_output_shapes
: *
dtype0
?
Adam/conv2d_24/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape: *(
shared_nameAdam/conv2d_24/kernel/m
?
+Adam/conv2d_24/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_24/kernel/m*&
_output_shapes
: *
dtype0
?
Adam/conv2d_24/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape: *&
shared_nameAdam/conv2d_24/bias/m
{
)Adam/conv2d_24/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_24/bias/m*
_output_shapes
: *
dtype0
?
Adam/conv2d_25/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:  *(
shared_nameAdam/conv2d_25/kernel/m
?
+Adam/conv2d_25/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_25/kernel/m*&
_output_shapes
:  *
dtype0
?
Adam/conv2d_25/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape: *&
shared_nameAdam/conv2d_25/bias/m
{
)Adam/conv2d_25/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_25/bias/m*
_output_shapes
: *
dtype0
?
Adam/conv2d_26/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape: @*(
shared_nameAdam/conv2d_26/kernel/m
?
+Adam/conv2d_26/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_26/kernel/m*&
_output_shapes
: @*
dtype0
?
Adam/conv2d_26/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*&
shared_nameAdam/conv2d_26/bias/m
{
)Adam/conv2d_26/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_26/bias/m*
_output_shapes
:@*
dtype0
?
Adam/dense_15/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?2*'
shared_nameAdam/dense_15/kernel/m
?
*Adam/dense_15/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_15/kernel/m*
_output_shapes
:	?2*
dtype0
?
Adam/dense_15/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_15/bias/m
y
(Adam/dense_15/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_15/bias/m*
_output_shapes
:*
dtype0
?
Adam/conv2d_24/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape: *(
shared_nameAdam/conv2d_24/kernel/v
?
+Adam/conv2d_24/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_24/kernel/v*&
_output_shapes
: *
dtype0
?
Adam/conv2d_24/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape: *&
shared_nameAdam/conv2d_24/bias/v
{
)Adam/conv2d_24/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_24/bias/v*
_output_shapes
: *
dtype0
?
Adam/conv2d_25/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:  *(
shared_nameAdam/conv2d_25/kernel/v
?
+Adam/conv2d_25/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_25/kernel/v*&
_output_shapes
:  *
dtype0
?
Adam/conv2d_25/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape: *&
shared_nameAdam/conv2d_25/bias/v
{
)Adam/conv2d_25/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_25/bias/v*
_output_shapes
: *
dtype0
?
Adam/conv2d_26/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape: @*(
shared_nameAdam/conv2d_26/kernel/v
?
+Adam/conv2d_26/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_26/kernel/v*&
_output_shapes
: @*
dtype0
?
Adam/conv2d_26/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:@*&
shared_nameAdam/conv2d_26/bias/v
{
)Adam/conv2d_26/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_26/bias/v*
_output_shapes
:@*
dtype0
?
Adam/dense_15/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?2*'
shared_nameAdam/dense_15/kernel/v
?
*Adam/dense_15/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_15/kernel/v*
_output_shapes
:	?2*
dtype0
?
Adam/dense_15/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_15/bias/v
y
(Adam/dense_15/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_15/bias/v*
_output_shapes
:*
dtype0

NoOpNoOp
?E
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*?E
value?EB?E B?E
?
layer_with_weights-0
layer-0
layer-1
layer-2
layer_with_weights-1
layer-3
layer-4
layer-5
layer_with_weights-2
layer-6
layer-7
	layer-8

layer-9
layer-10
layer-11
layer_with_weights-3
layer-12
layer-13
	optimizer
	variables
regularization_losses
trainable_variables
	keras_api

signatures
h

kernel
bias
	variables
regularization_losses
trainable_variables
	keras_api
R
	variables
regularization_losses
trainable_variables
	keras_api
R
	variables
 regularization_losses
!trainable_variables
"	keras_api
h

#kernel
$bias
%	variables
&regularization_losses
'trainable_variables
(	keras_api
R
)	variables
*regularization_losses
+trainable_variables
,	keras_api
R
-	variables
.regularization_losses
/trainable_variables
0	keras_api
h

1kernel
2bias
3	variables
4regularization_losses
5trainable_variables
6	keras_api
R
7	variables
8regularization_losses
9trainable_variables
:	keras_api
R
;	variables
<regularization_losses
=trainable_variables
>	keras_api
R
?	variables
@regularization_losses
Atrainable_variables
B	keras_api
R
C	variables
Dregularization_losses
Etrainable_variables
F	keras_api
R
G	variables
Hregularization_losses
Itrainable_variables
J	keras_api
h

Kkernel
Lbias
M	variables
Nregularization_losses
Otrainable_variables
P	keras_api
R
Q	variables
Rregularization_losses
Strainable_variables
T	keras_api
?
Uiter

Vbeta_1

Wbeta_2
	Xdecay
Ylearning_ratem?m?#m?$m?1m?2m?Km?Lm?v?v?#v?$v?1v?2v?Kv?Lv?
8
0
1
#2
$3
14
25
K6
L7
 
8
0
1
#2
$3
14
25
K6
L7
?
	variables
Zlayer_metrics
[layer_regularization_losses
\metrics
regularization_losses

]layers
trainable_variables
^non_trainable_variables
 
\Z
VARIABLE_VALUEconv2d_24/kernel6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_24/bias4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUE

0
1
 

0
1
?
	variables
_layer_metrics
`layer_regularization_losses
ametrics
regularization_losses

blayers
trainable_variables
cnon_trainable_variables
 
 
 
?
	variables
dlayer_metrics
elayer_regularization_losses
fmetrics
regularization_losses

glayers
trainable_variables
hnon_trainable_variables
 
 
 
?
	variables
ilayer_metrics
jlayer_regularization_losses
kmetrics
 regularization_losses

llayers
!trainable_variables
mnon_trainable_variables
\Z
VARIABLE_VALUEconv2d_25/kernel6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_25/bias4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUE

#0
$1
 

#0
$1
?
%	variables
nlayer_metrics
olayer_regularization_losses
pmetrics
&regularization_losses

qlayers
'trainable_variables
rnon_trainable_variables
 
 
 
?
)	variables
slayer_metrics
tlayer_regularization_losses
umetrics
*regularization_losses

vlayers
+trainable_variables
wnon_trainable_variables
 
 
 
?
-	variables
xlayer_metrics
ylayer_regularization_losses
zmetrics
.regularization_losses

{layers
/trainable_variables
|non_trainable_variables
\Z
VARIABLE_VALUEconv2d_26/kernel6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_26/bias4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUE

10
21
 

10
21
?
3	variables
}layer_metrics
~layer_regularization_losses
metrics
4regularization_losses
?layers
5trainable_variables
?non_trainable_variables
 
 
 
?
7	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
8regularization_losses
?layers
9trainable_variables
?non_trainable_variables
 
 
 
?
;	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
<regularization_losses
?layers
=trainable_variables
?non_trainable_variables
 
 
 
?
?	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
@regularization_losses
?layers
Atrainable_variables
?non_trainable_variables
 
 
 
?
C	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Dregularization_losses
?layers
Etrainable_variables
?non_trainable_variables
 
 
 
?
G	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Hregularization_losses
?layers
Itrainable_variables
?non_trainable_variables
[Y
VARIABLE_VALUEdense_15/kernel6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEdense_15/bias4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUE

K0
L1
 

K0
L1
?
M	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Nregularization_losses
?layers
Otrainable_variables
?non_trainable_variables
 
 
 
?
Q	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Rregularization_losses
?layers
Strainable_variables
?non_trainable_variables
HF
VARIABLE_VALUE	Adam/iter)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_1+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_2+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUE
JH
VARIABLE_VALUE
Adam/decay*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEAdam/learning_rate2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUE
 
 

?0
?1
f
0
1
2
3
4
5
6
7
	8

9
10
11
12
13
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
8

?total

?count
?	variables
?	keras_api
I

?total

?count
?
_fn_kwargs
?	variables
?	keras_api
OM
VARIABLE_VALUEtotal4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUE
OM
VARIABLE_VALUEcount4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUE

?0
?1

?	variables
QO
VARIABLE_VALUEtotal_14keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUE
QO
VARIABLE_VALUEcount_14keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUE
 

?0
?1

?	variables
}
VARIABLE_VALUEAdam/conv2d_24/kernel/mRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_24/bias/mPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_25/kernel/mRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_25/bias/mPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_26/kernel/mRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_26/bias/mPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_15/kernel/mRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_15/bias/mPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_24/kernel/vRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_24/bias/vPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_25/kernel/vRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_25/bias/vPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_26/kernel/vRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_26/bias/vPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_15/kernel/vRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_15/bias/vPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?
serving_default_conv2d_24_inputPlaceholder*/
_output_shapes
:?????????dd*
dtype0*$
shape:?????????dd
?
StatefulPartitionedCallStatefulPartitionedCallserving_default_conv2d_24_inputconv2d_24/kernelconv2d_24/biasconv2d_25/kernelconv2d_25/biasconv2d_26/kernelconv2d_26/biasdense_15/kerneldense_15/bias*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *+
f&R$
"__inference_signature_wrapper_9068
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
?
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename$conv2d_24/kernel/Read/ReadVariableOp"conv2d_24/bias/Read/ReadVariableOp$conv2d_25/kernel/Read/ReadVariableOp"conv2d_25/bias/Read/ReadVariableOp$conv2d_26/kernel/Read/ReadVariableOp"conv2d_26/bias/Read/ReadVariableOp#dense_15/kernel/Read/ReadVariableOp!dense_15/bias/Read/ReadVariableOpAdam/iter/Read/ReadVariableOpAdam/beta_1/Read/ReadVariableOpAdam/beta_2/Read/ReadVariableOpAdam/decay/Read/ReadVariableOp&Adam/learning_rate/Read/ReadVariableOptotal/Read/ReadVariableOpcount/Read/ReadVariableOptotal_1/Read/ReadVariableOpcount_1/Read/ReadVariableOp+Adam/conv2d_24/kernel/m/Read/ReadVariableOp)Adam/conv2d_24/bias/m/Read/ReadVariableOp+Adam/conv2d_25/kernel/m/Read/ReadVariableOp)Adam/conv2d_25/bias/m/Read/ReadVariableOp+Adam/conv2d_26/kernel/m/Read/ReadVariableOp)Adam/conv2d_26/bias/m/Read/ReadVariableOp*Adam/dense_15/kernel/m/Read/ReadVariableOp(Adam/dense_15/bias/m/Read/ReadVariableOp+Adam/conv2d_24/kernel/v/Read/ReadVariableOp)Adam/conv2d_24/bias/v/Read/ReadVariableOp+Adam/conv2d_25/kernel/v/Read/ReadVariableOp)Adam/conv2d_25/bias/v/Read/ReadVariableOp+Adam/conv2d_26/kernel/v/Read/ReadVariableOp)Adam/conv2d_26/bias/v/Read/ReadVariableOp*Adam/dense_15/kernel/v/Read/ReadVariableOp(Adam/dense_15/bias/v/Read/ReadVariableOpConst*.
Tin'
%2#	*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *&
f!R
__inference__traced_save_9481
?
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenameconv2d_24/kernelconv2d_24/biasconv2d_25/kernelconv2d_25/biasconv2d_26/kernelconv2d_26/biasdense_15/kerneldense_15/bias	Adam/iterAdam/beta_1Adam/beta_2
Adam/decayAdam/learning_ratetotalcounttotal_1count_1Adam/conv2d_24/kernel/mAdam/conv2d_24/bias/mAdam/conv2d_25/kernel/mAdam/conv2d_25/bias/mAdam/conv2d_26/kernel/mAdam/conv2d_26/bias/mAdam/dense_15/kernel/mAdam/dense_15/bias/mAdam/conv2d_24/kernel/vAdam/conv2d_24/bias/vAdam/conv2d_25/kernel/vAdam/conv2d_25/bias/vAdam/conv2d_26/kernel/vAdam/conv2d_26/bias/vAdam/dense_15/kernel/vAdam/dense_15/bias/v*-
Tin&
$2"*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *)
f$R"
 __inference__traced_restore_9590??
?
a
(__inference_dropout_5_layer_call_fn_9325

inputs
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88342
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????222
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
?
+__inference_sequential_8_layer_call_fn_9037
conv2d_24_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_24_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_sequential_8_layer_call_and_return_conditional_losses_90182
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?	
?
B__inference_dense_15_layer_call_and_return_conditional_losses_8862

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	?2*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????2::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
c
G__inference_activation_25_layer_call_and_return_conditional_losses_9219

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????bb 2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????bb :W S
/
_output_shapes
:?????????bb 
 
_user_specified_nameinputs
?
c
G__inference_activation_29_layer_call_and_return_conditional_losses_9354

inputs
identityW
SigmoidSigmoidinputs*
T0*'
_output_shapes
:?????????2	
Sigmoid_
IdentityIdentitySigmoid:y:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
"__inference_signature_wrapper_9068
conv2d_24_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_24_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *(
f#R!
__inference__wrapped_model_86352
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?	
?
C__inference_conv2d_25_layer_call_and_return_conditional_losses_8725

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:  *
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// *
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
: *
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// 2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????11 ::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????11 
 
_user_specified_nameinputs
?	
?
C__inference_conv2d_24_layer_call_and_return_conditional_losses_9205

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
: *
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb *
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
: *
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb 2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????dd::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?6
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_8963

inputs
conv2d_24_8932
conv2d_24_8934
conv2d_25_8939
conv2d_25_8941
conv2d_26_8946
conv2d_26_8948
dense_15_8956
dense_15_8958
identity??!conv2d_24/StatefulPartitionedCall?!conv2d_25/StatefulPartitionedCall?!conv2d_26/StatefulPartitionedCall? dense_15/StatefulPartitionedCall?!dropout_5/StatefulPartitionedCall?
!conv2d_24/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_24_8932conv2d_24_8934*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_24_layer_call_and_return_conditional_losses_86852#
!conv2d_24/StatefulPartitionedCall?
activation_25/PartitionedCallPartitionedCall*conv2d_24/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_25_layer_call_and_return_conditional_losses_87062
activation_25/PartitionedCall?
 max_pooling2d_24/PartitionedCallPartitionedCall&activation_25/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????11 * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_86412"
 max_pooling2d_24/PartitionedCall?
!conv2d_25/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_24/PartitionedCall:output:0conv2d_25_8939conv2d_25_8941*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_25_layer_call_and_return_conditional_losses_87252#
!conv2d_25/StatefulPartitionedCall?
activation_26/PartitionedCallPartitionedCall*conv2d_25/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_26_layer_call_and_return_conditional_losses_87462
activation_26/PartitionedCall?
 max_pooling2d_25/PartitionedCallPartitionedCall&activation_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:????????? * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_86532"
 max_pooling2d_25/PartitionedCall?
!conv2d_26/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_25/PartitionedCall:output:0conv2d_26_8946conv2d_26_8948*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_26_layer_call_and_return_conditional_losses_87652#
!conv2d_26/StatefulPartitionedCall?
activation_27/PartitionedCallPartitionedCall*conv2d_26/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_27_layer_call_and_return_conditional_losses_87862
activation_27/PartitionedCall?
 max_pooling2d_26/PartitionedCallPartitionedCall&activation_27/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????

@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_86652"
 max_pooling2d_26/PartitionedCall?
flatten_8/PartitionedCallPartitionedCall)max_pooling2d_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_flatten_8_layer_call_and_return_conditional_losses_88012
flatten_8/PartitionedCall?
activation_28/PartitionedCallPartitionedCall"flatten_8/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_28_layer_call_and_return_conditional_losses_88142
activation_28/PartitionedCall?
!dropout_5/StatefulPartitionedCallStatefulPartitionedCall&activation_28/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88342#
!dropout_5/StatefulPartitionedCall?
 dense_15/StatefulPartitionedCallStatefulPartitionedCall*dropout_5/StatefulPartitionedCall:output:0dense_15_8956dense_15_8958*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *K
fFRD
B__inference_dense_15_layer_call_and_return_conditional_losses_88622"
 dense_15/StatefulPartitionedCall?
activation_29/PartitionedCallPartitionedCall)dense_15/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_29_layer_call_and_return_conditional_losses_88832
activation_29/PartitionedCall?
IdentityIdentity&activation_29/PartitionedCall:output:0"^conv2d_24/StatefulPartitionedCall"^conv2d_25/StatefulPartitionedCall"^conv2d_26/StatefulPartitionedCall!^dense_15/StatefulPartitionedCall"^dropout_5/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2F
!conv2d_24/StatefulPartitionedCall!conv2d_24/StatefulPartitionedCall2F
!conv2d_25/StatefulPartitionedCall!conv2d_25/StatefulPartitionedCall2F
!conv2d_26/StatefulPartitionedCall!conv2d_26/StatefulPartitionedCall2D
 dense_15/StatefulPartitionedCall dense_15/StatefulPartitionedCall2F
!dropout_5/StatefulPartitionedCall!dropout_5/StatefulPartitionedCall:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
H
,__inference_activation_25_layer_call_fn_9224

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_25_layer_call_and_return_conditional_losses_87062
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????bb :W S
/
_output_shapes
:?????????bb 
 
_user_specified_nameinputs
?
a
C__inference_dropout_5_layer_call_and_return_conditional_losses_9320

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????22

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????22

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
f
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_8641

inputs
identity?
MaxPoolMaxPoolinputs*J
_output_shapes8
6:4????????????????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
IdentityIdentityMaxPool:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?	
?
C__inference_conv2d_26_layer_call_and_return_conditional_losses_9263

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
: @*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:????????? ::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:????????? 
 
_user_specified_nameinputs
?
H
,__inference_activation_29_layer_call_fn_9359

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_29_layer_call_and_return_conditional_losses_88832
PartitionedCalll
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
c
G__inference_activation_29_layer_call_and_return_conditional_losses_8883

inputs
identityW
SigmoidSigmoidinputs*
T0*'
_output_shapes
:?????????2	
Sigmoid_
IdentityIdentitySigmoid:y:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
|
'__inference_dense_15_layer_call_fn_9349

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *K
fFRD
B__inference_dense_15_layer_call_and_return_conditional_losses_88622
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????2::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?<
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_9114

inputs,
(conv2d_24_conv2d_readvariableop_resource-
)conv2d_24_biasadd_readvariableop_resource,
(conv2d_25_conv2d_readvariableop_resource-
)conv2d_25_biasadd_readvariableop_resource,
(conv2d_26_conv2d_readvariableop_resource-
)conv2d_26_biasadd_readvariableop_resource+
'dense_15_matmul_readvariableop_resource,
(dense_15_biasadd_readvariableop_resource
identity?? conv2d_24/BiasAdd/ReadVariableOp?conv2d_24/Conv2D/ReadVariableOp? conv2d_25/BiasAdd/ReadVariableOp?conv2d_25/Conv2D/ReadVariableOp? conv2d_26/BiasAdd/ReadVariableOp?conv2d_26/Conv2D/ReadVariableOp?dense_15/BiasAdd/ReadVariableOp?dense_15/MatMul/ReadVariableOp?
conv2d_24/Conv2D/ReadVariableOpReadVariableOp(conv2d_24_conv2d_readvariableop_resource*&
_output_shapes
: *
dtype02!
conv2d_24/Conv2D/ReadVariableOp?
conv2d_24/Conv2DConv2Dinputs'conv2d_24/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb *
paddingVALID*
strides
2
conv2d_24/Conv2D?
 conv2d_24/BiasAdd/ReadVariableOpReadVariableOp)conv2d_24_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02"
 conv2d_24/BiasAdd/ReadVariableOp?
conv2d_24/BiasAddBiasAddconv2d_24/Conv2D:output:0(conv2d_24/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb 2
conv2d_24/BiasAdd?
activation_25/ReluReluconv2d_24/BiasAdd:output:0*
T0*/
_output_shapes
:?????????bb 2
activation_25/Relu?
max_pooling2d_24/MaxPoolMaxPool activation_25/Relu:activations:0*/
_output_shapes
:?????????11 *
ksize
*
paddingVALID*
strides
2
max_pooling2d_24/MaxPool?
conv2d_25/Conv2D/ReadVariableOpReadVariableOp(conv2d_25_conv2d_readvariableop_resource*&
_output_shapes
:  *
dtype02!
conv2d_25/Conv2D/ReadVariableOp?
conv2d_25/Conv2DConv2D!max_pooling2d_24/MaxPool:output:0'conv2d_25/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// *
paddingVALID*
strides
2
conv2d_25/Conv2D?
 conv2d_25/BiasAdd/ReadVariableOpReadVariableOp)conv2d_25_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02"
 conv2d_25/BiasAdd/ReadVariableOp?
conv2d_25/BiasAddBiasAddconv2d_25/Conv2D:output:0(conv2d_25/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// 2
conv2d_25/BiasAdd?
activation_26/ReluReluconv2d_25/BiasAdd:output:0*
T0*/
_output_shapes
:?????????// 2
activation_26/Relu?
max_pooling2d_25/MaxPoolMaxPool activation_26/Relu:activations:0*/
_output_shapes
:????????? *
ksize
*
paddingVALID*
strides
2
max_pooling2d_25/MaxPool?
conv2d_26/Conv2D/ReadVariableOpReadVariableOp(conv2d_26_conv2d_readvariableop_resource*&
_output_shapes
: @*
dtype02!
conv2d_26/Conv2D/ReadVariableOp?
conv2d_26/Conv2DConv2D!max_pooling2d_25/MaxPool:output:0'conv2d_26/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@*
paddingVALID*
strides
2
conv2d_26/Conv2D?
 conv2d_26/BiasAdd/ReadVariableOpReadVariableOp)conv2d_26_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype02"
 conv2d_26/BiasAdd/ReadVariableOp?
conv2d_26/BiasAddBiasAddconv2d_26/Conv2D:output:0(conv2d_26/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@2
conv2d_26/BiasAdd?
activation_27/ReluReluconv2d_26/BiasAdd:output:0*
T0*/
_output_shapes
:?????????@2
activation_27/Relu?
max_pooling2d_26/MaxPoolMaxPool activation_27/Relu:activations:0*/
_output_shapes
:?????????

@*
ksize
*
paddingVALID*
strides
2
max_pooling2d_26/MaxPools
flatten_8/ConstConst*
_output_shapes
:*
dtype0*
valueB"????   2
flatten_8/Const?
flatten_8/ReshapeReshape!max_pooling2d_26/MaxPool:output:0flatten_8/Const:output:0*
T0*(
_output_shapes
:??????????22
flatten_8/Reshape
activation_28/ReluReluflatten_8/Reshape:output:0*
T0*(
_output_shapes
:??????????22
activation_28/Reluw
dropout_5/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout_5/dropout/Const?
dropout_5/dropout/MulMul activation_28/Relu:activations:0 dropout_5/dropout/Const:output:0*
T0*(
_output_shapes
:??????????22
dropout_5/dropout/Mul?
dropout_5/dropout/ShapeShape activation_28/Relu:activations:0*
T0*
_output_shapes
:2
dropout_5/dropout/Shape?
.dropout_5/dropout/random_uniform/RandomUniformRandomUniform dropout_5/dropout/Shape:output:0*
T0*(
_output_shapes
:??????????2*
dtype020
.dropout_5/dropout/random_uniform/RandomUniform?
 dropout_5/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2"
 dropout_5/dropout/GreaterEqual/y?
dropout_5/dropout/GreaterEqualGreaterEqual7dropout_5/dropout/random_uniform/RandomUniform:output:0)dropout_5/dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????22 
dropout_5/dropout/GreaterEqual?
dropout_5/dropout/CastCast"dropout_5/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????22
dropout_5/dropout/Cast?
dropout_5/dropout/Mul_1Muldropout_5/dropout/Mul:z:0dropout_5/dropout/Cast:y:0*
T0*(
_output_shapes
:??????????22
dropout_5/dropout/Mul_1?
dense_15/MatMul/ReadVariableOpReadVariableOp'dense_15_matmul_readvariableop_resource*
_output_shapes
:	?2*
dtype02 
dense_15/MatMul/ReadVariableOp?
dense_15/MatMulMatMuldropout_5/dropout/Mul_1:z:0&dense_15/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_15/MatMul?
dense_15/BiasAdd/ReadVariableOpReadVariableOp(dense_15_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_15/BiasAdd/ReadVariableOp?
dense_15/BiasAddBiasAdddense_15/MatMul:product:0'dense_15/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_15/BiasAdd?
activation_29/SigmoidSigmoiddense_15/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_29/Sigmoid?
IdentityIdentityactivation_29/Sigmoid:y:0!^conv2d_24/BiasAdd/ReadVariableOp ^conv2d_24/Conv2D/ReadVariableOp!^conv2d_25/BiasAdd/ReadVariableOp ^conv2d_25/Conv2D/ReadVariableOp!^conv2d_26/BiasAdd/ReadVariableOp ^conv2d_26/Conv2D/ReadVariableOp ^dense_15/BiasAdd/ReadVariableOp^dense_15/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2D
 conv2d_24/BiasAdd/ReadVariableOp conv2d_24/BiasAdd/ReadVariableOp2B
conv2d_24/Conv2D/ReadVariableOpconv2d_24/Conv2D/ReadVariableOp2D
 conv2d_25/BiasAdd/ReadVariableOp conv2d_25/BiasAdd/ReadVariableOp2B
conv2d_25/Conv2D/ReadVariableOpconv2d_25/Conv2D/ReadVariableOp2D
 conv2d_26/BiasAdd/ReadVariableOp conv2d_26/BiasAdd/ReadVariableOp2B
conv2d_26/Conv2D/ReadVariableOpconv2d_26/Conv2D/ReadVariableOp2B
dense_15/BiasAdd/ReadVariableOpdense_15/BiasAdd/ReadVariableOp2@
dense_15/MatMul/ReadVariableOpdense_15/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
_
C__inference_flatten_8_layer_call_and_return_conditional_losses_8801

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"????   2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????22	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????

@:W S
/
_output_shapes
:?????????

@
 
_user_specified_nameinputs
?7
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_8892
conv2d_24_input
conv2d_24_8696
conv2d_24_8698
conv2d_25_8736
conv2d_25_8738
conv2d_26_8776
conv2d_26_8778
dense_15_8873
dense_15_8875
identity??!conv2d_24/StatefulPartitionedCall?!conv2d_25/StatefulPartitionedCall?!conv2d_26/StatefulPartitionedCall? dense_15/StatefulPartitionedCall?!dropout_5/StatefulPartitionedCall?
!conv2d_24/StatefulPartitionedCallStatefulPartitionedCallconv2d_24_inputconv2d_24_8696conv2d_24_8698*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_24_layer_call_and_return_conditional_losses_86852#
!conv2d_24/StatefulPartitionedCall?
activation_25/PartitionedCallPartitionedCall*conv2d_24/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_25_layer_call_and_return_conditional_losses_87062
activation_25/PartitionedCall?
 max_pooling2d_24/PartitionedCallPartitionedCall&activation_25/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????11 * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_86412"
 max_pooling2d_24/PartitionedCall?
!conv2d_25/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_24/PartitionedCall:output:0conv2d_25_8736conv2d_25_8738*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_25_layer_call_and_return_conditional_losses_87252#
!conv2d_25/StatefulPartitionedCall?
activation_26/PartitionedCallPartitionedCall*conv2d_25/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_26_layer_call_and_return_conditional_losses_87462
activation_26/PartitionedCall?
 max_pooling2d_25/PartitionedCallPartitionedCall&activation_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:????????? * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_86532"
 max_pooling2d_25/PartitionedCall?
!conv2d_26/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_25/PartitionedCall:output:0conv2d_26_8776conv2d_26_8778*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_26_layer_call_and_return_conditional_losses_87652#
!conv2d_26/StatefulPartitionedCall?
activation_27/PartitionedCallPartitionedCall*conv2d_26/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_27_layer_call_and_return_conditional_losses_87862
activation_27/PartitionedCall?
 max_pooling2d_26/PartitionedCallPartitionedCall&activation_27/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????

@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_86652"
 max_pooling2d_26/PartitionedCall?
flatten_8/PartitionedCallPartitionedCall)max_pooling2d_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_flatten_8_layer_call_and_return_conditional_losses_88012
flatten_8/PartitionedCall?
activation_28/PartitionedCallPartitionedCall"flatten_8/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_28_layer_call_and_return_conditional_losses_88142
activation_28/PartitionedCall?
!dropout_5/StatefulPartitionedCallStatefulPartitionedCall&activation_28/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88342#
!dropout_5/StatefulPartitionedCall?
 dense_15/StatefulPartitionedCallStatefulPartitionedCall*dropout_5/StatefulPartitionedCall:output:0dense_15_8873dense_15_8875*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *K
fFRD
B__inference_dense_15_layer_call_and_return_conditional_losses_88622"
 dense_15/StatefulPartitionedCall?
activation_29/PartitionedCallPartitionedCall)dense_15/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_29_layer_call_and_return_conditional_losses_88832
activation_29/PartitionedCall?
IdentityIdentity&activation_29/PartitionedCall:output:0"^conv2d_24/StatefulPartitionedCall"^conv2d_25/StatefulPartitionedCall"^conv2d_26/StatefulPartitionedCall!^dense_15/StatefulPartitionedCall"^dropout_5/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2F
!conv2d_24/StatefulPartitionedCall!conv2d_24/StatefulPartitionedCall2F
!conv2d_25/StatefulPartitionedCall!conv2d_25/StatefulPartitionedCall2F
!conv2d_26/StatefulPartitionedCall!conv2d_26/StatefulPartitionedCall2D
 dense_15/StatefulPartitionedCall dense_15/StatefulPartitionedCall2F
!dropout_5/StatefulPartitionedCall!dropout_5/StatefulPartitionedCall:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?
?
+__inference_sequential_8_layer_call_fn_8982
conv2d_24_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_24_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_sequential_8_layer_call_and_return_conditional_losses_89632
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?
c
G__inference_activation_28_layer_call_and_return_conditional_losses_9298

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????22
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?5
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_8926
conv2d_24_input
conv2d_24_8895
conv2d_24_8897
conv2d_25_8902
conv2d_25_8904
conv2d_26_8909
conv2d_26_8911
dense_15_8919
dense_15_8921
identity??!conv2d_24/StatefulPartitionedCall?!conv2d_25/StatefulPartitionedCall?!conv2d_26/StatefulPartitionedCall? dense_15/StatefulPartitionedCall?
!conv2d_24/StatefulPartitionedCallStatefulPartitionedCallconv2d_24_inputconv2d_24_8895conv2d_24_8897*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_24_layer_call_and_return_conditional_losses_86852#
!conv2d_24/StatefulPartitionedCall?
activation_25/PartitionedCallPartitionedCall*conv2d_24/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_25_layer_call_and_return_conditional_losses_87062
activation_25/PartitionedCall?
 max_pooling2d_24/PartitionedCallPartitionedCall&activation_25/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????11 * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_86412"
 max_pooling2d_24/PartitionedCall?
!conv2d_25/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_24/PartitionedCall:output:0conv2d_25_8902conv2d_25_8904*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_25_layer_call_and_return_conditional_losses_87252#
!conv2d_25/StatefulPartitionedCall?
activation_26/PartitionedCallPartitionedCall*conv2d_25/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_26_layer_call_and_return_conditional_losses_87462
activation_26/PartitionedCall?
 max_pooling2d_25/PartitionedCallPartitionedCall&activation_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:????????? * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_86532"
 max_pooling2d_25/PartitionedCall?
!conv2d_26/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_25/PartitionedCall:output:0conv2d_26_8909conv2d_26_8911*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_26_layer_call_and_return_conditional_losses_87652#
!conv2d_26/StatefulPartitionedCall?
activation_27/PartitionedCallPartitionedCall*conv2d_26/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_27_layer_call_and_return_conditional_losses_87862
activation_27/PartitionedCall?
 max_pooling2d_26/PartitionedCallPartitionedCall&activation_27/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????

@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_86652"
 max_pooling2d_26/PartitionedCall?
flatten_8/PartitionedCallPartitionedCall)max_pooling2d_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_flatten_8_layer_call_and_return_conditional_losses_88012
flatten_8/PartitionedCall?
activation_28/PartitionedCallPartitionedCall"flatten_8/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_28_layer_call_and_return_conditional_losses_88142
activation_28/PartitionedCall?
dropout_5/PartitionedCallPartitionedCall&activation_28/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88392
dropout_5/PartitionedCall?
 dense_15/StatefulPartitionedCallStatefulPartitionedCall"dropout_5/PartitionedCall:output:0dense_15_8919dense_15_8921*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *K
fFRD
B__inference_dense_15_layer_call_and_return_conditional_losses_88622"
 dense_15/StatefulPartitionedCall?
activation_29/PartitionedCallPartitionedCall)dense_15/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_29_layer_call_and_return_conditional_losses_88832
activation_29/PartitionedCall?
IdentityIdentity&activation_29/PartitionedCall:output:0"^conv2d_24/StatefulPartitionedCall"^conv2d_25/StatefulPartitionedCall"^conv2d_26/StatefulPartitionedCall!^dense_15/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2F
!conv2d_24/StatefulPartitionedCall!conv2d_24/StatefulPartitionedCall2F
!conv2d_25/StatefulPartitionedCall!conv2d_25/StatefulPartitionedCall2F
!conv2d_26/StatefulPartitionedCall!conv2d_26/StatefulPartitionedCall2D
 dense_15/StatefulPartitionedCall dense_15/StatefulPartitionedCall:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?
a
C__inference_dropout_5_layer_call_and_return_conditional_losses_8839

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????22

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????22

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
??
?
 __inference__traced_restore_9590
file_prefix%
!assignvariableop_conv2d_24_kernel%
!assignvariableop_1_conv2d_24_bias'
#assignvariableop_2_conv2d_25_kernel%
!assignvariableop_3_conv2d_25_bias'
#assignvariableop_4_conv2d_26_kernel%
!assignvariableop_5_conv2d_26_bias&
"assignvariableop_6_dense_15_kernel$
 assignvariableop_7_dense_15_bias 
assignvariableop_8_adam_iter"
assignvariableop_9_adam_beta_1#
assignvariableop_10_adam_beta_2"
assignvariableop_11_adam_decay*
&assignvariableop_12_adam_learning_rate
assignvariableop_13_total
assignvariableop_14_count
assignvariableop_15_total_1
assignvariableop_16_count_1/
+assignvariableop_17_adam_conv2d_24_kernel_m-
)assignvariableop_18_adam_conv2d_24_bias_m/
+assignvariableop_19_adam_conv2d_25_kernel_m-
)assignvariableop_20_adam_conv2d_25_bias_m/
+assignvariableop_21_adam_conv2d_26_kernel_m-
)assignvariableop_22_adam_conv2d_26_bias_m.
*assignvariableop_23_adam_dense_15_kernel_m,
(assignvariableop_24_adam_dense_15_bias_m/
+assignvariableop_25_adam_conv2d_24_kernel_v-
)assignvariableop_26_adam_conv2d_24_bias_v/
+assignvariableop_27_adam_conv2d_25_kernel_v-
)assignvariableop_28_adam_conv2d_25_bias_v/
+assignvariableop_29_adam_conv2d_26_kernel_v-
)assignvariableop_30_adam_conv2d_26_bias_v.
*assignvariableop_31_adam_dense_15_kernel_v,
(assignvariableop_32_adam_dense_15_bias_v
identity_34??AssignVariableOp?AssignVariableOp_1?AssignVariableOp_10?AssignVariableOp_11?AssignVariableOp_12?AssignVariableOp_13?AssignVariableOp_14?AssignVariableOp_15?AssignVariableOp_16?AssignVariableOp_17?AssignVariableOp_18?AssignVariableOp_19?AssignVariableOp_2?AssignVariableOp_20?AssignVariableOp_21?AssignVariableOp_22?AssignVariableOp_23?AssignVariableOp_24?AssignVariableOp_25?AssignVariableOp_26?AssignVariableOp_27?AssignVariableOp_28?AssignVariableOp_29?AssignVariableOp_3?AssignVariableOp_30?AssignVariableOp_31?AssignVariableOp_32?AssignVariableOp_4?AssignVariableOp_5?AssignVariableOp_6?AssignVariableOp_7?AssignVariableOp_8?AssignVariableOp_9?
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:"*
dtype0*?
value?B?"B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names?
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:"*
dtype0*W
valueNBL"B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slices?
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*?
_output_shapes?
?::::::::::::::::::::::::::::::::::*0
dtypes&
$2"	2
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identity?
AssignVariableOpAssignVariableOp!assignvariableop_conv2d_24_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1?
AssignVariableOp_1AssignVariableOp!assignvariableop_1_conv2d_24_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2?
AssignVariableOp_2AssignVariableOp#assignvariableop_2_conv2d_25_kernelIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3?
AssignVariableOp_3AssignVariableOp!assignvariableop_3_conv2d_25_biasIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4?
AssignVariableOp_4AssignVariableOp#assignvariableop_4_conv2d_26_kernelIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5?
AssignVariableOp_5AssignVariableOp!assignvariableop_5_conv2d_26_biasIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6?
AssignVariableOp_6AssignVariableOp"assignvariableop_6_dense_15_kernelIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7?
AssignVariableOp_7AssignVariableOp assignvariableop_7_dense_15_biasIdentity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0	*
_output_shapes
:2

Identity_8?
AssignVariableOp_8AssignVariableOpassignvariableop_8_adam_iterIdentity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype0	2
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9?
AssignVariableOp_9AssignVariableOpassignvariableop_9_adam_beta_1Identity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10?
AssignVariableOp_10AssignVariableOpassignvariableop_10_adam_beta_2Identity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11?
AssignVariableOp_11AssignVariableOpassignvariableop_11_adam_decayIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12?
AssignVariableOp_12AssignVariableOp&assignvariableop_12_adam_learning_rateIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13?
AssignVariableOp_13AssignVariableOpassignvariableop_13_totalIdentity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_13n
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:2
Identity_14?
AssignVariableOp_14AssignVariableOpassignvariableop_14_countIdentity_14:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_14n
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:2
Identity_15?
AssignVariableOp_15AssignVariableOpassignvariableop_15_total_1Identity_15:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_15n
Identity_16IdentityRestoreV2:tensors:16"/device:CPU:0*
T0*
_output_shapes
:2
Identity_16?
AssignVariableOp_16AssignVariableOpassignvariableop_16_count_1Identity_16:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_16n
Identity_17IdentityRestoreV2:tensors:17"/device:CPU:0*
T0*
_output_shapes
:2
Identity_17?
AssignVariableOp_17AssignVariableOp+assignvariableop_17_adam_conv2d_24_kernel_mIdentity_17:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_17n
Identity_18IdentityRestoreV2:tensors:18"/device:CPU:0*
T0*
_output_shapes
:2
Identity_18?
AssignVariableOp_18AssignVariableOp)assignvariableop_18_adam_conv2d_24_bias_mIdentity_18:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_18n
Identity_19IdentityRestoreV2:tensors:19"/device:CPU:0*
T0*
_output_shapes
:2
Identity_19?
AssignVariableOp_19AssignVariableOp+assignvariableop_19_adam_conv2d_25_kernel_mIdentity_19:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_19n
Identity_20IdentityRestoreV2:tensors:20"/device:CPU:0*
T0*
_output_shapes
:2
Identity_20?
AssignVariableOp_20AssignVariableOp)assignvariableop_20_adam_conv2d_25_bias_mIdentity_20:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_20n
Identity_21IdentityRestoreV2:tensors:21"/device:CPU:0*
T0*
_output_shapes
:2
Identity_21?
AssignVariableOp_21AssignVariableOp+assignvariableop_21_adam_conv2d_26_kernel_mIdentity_21:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_21n
Identity_22IdentityRestoreV2:tensors:22"/device:CPU:0*
T0*
_output_shapes
:2
Identity_22?
AssignVariableOp_22AssignVariableOp)assignvariableop_22_adam_conv2d_26_bias_mIdentity_22:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_22n
Identity_23IdentityRestoreV2:tensors:23"/device:CPU:0*
T0*
_output_shapes
:2
Identity_23?
AssignVariableOp_23AssignVariableOp*assignvariableop_23_adam_dense_15_kernel_mIdentity_23:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_23n
Identity_24IdentityRestoreV2:tensors:24"/device:CPU:0*
T0*
_output_shapes
:2
Identity_24?
AssignVariableOp_24AssignVariableOp(assignvariableop_24_adam_dense_15_bias_mIdentity_24:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_24n
Identity_25IdentityRestoreV2:tensors:25"/device:CPU:0*
T0*
_output_shapes
:2
Identity_25?
AssignVariableOp_25AssignVariableOp+assignvariableop_25_adam_conv2d_24_kernel_vIdentity_25:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_25n
Identity_26IdentityRestoreV2:tensors:26"/device:CPU:0*
T0*
_output_shapes
:2
Identity_26?
AssignVariableOp_26AssignVariableOp)assignvariableop_26_adam_conv2d_24_bias_vIdentity_26:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_26n
Identity_27IdentityRestoreV2:tensors:27"/device:CPU:0*
T0*
_output_shapes
:2
Identity_27?
AssignVariableOp_27AssignVariableOp+assignvariableop_27_adam_conv2d_25_kernel_vIdentity_27:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_27n
Identity_28IdentityRestoreV2:tensors:28"/device:CPU:0*
T0*
_output_shapes
:2
Identity_28?
AssignVariableOp_28AssignVariableOp)assignvariableop_28_adam_conv2d_25_bias_vIdentity_28:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_28n
Identity_29IdentityRestoreV2:tensors:29"/device:CPU:0*
T0*
_output_shapes
:2
Identity_29?
AssignVariableOp_29AssignVariableOp+assignvariableop_29_adam_conv2d_26_kernel_vIdentity_29:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_29n
Identity_30IdentityRestoreV2:tensors:30"/device:CPU:0*
T0*
_output_shapes
:2
Identity_30?
AssignVariableOp_30AssignVariableOp)assignvariableop_30_adam_conv2d_26_bias_vIdentity_30:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_30n
Identity_31IdentityRestoreV2:tensors:31"/device:CPU:0*
T0*
_output_shapes
:2
Identity_31?
AssignVariableOp_31AssignVariableOp*assignvariableop_31_adam_dense_15_kernel_vIdentity_31:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_31n
Identity_32IdentityRestoreV2:tensors:32"/device:CPU:0*
T0*
_output_shapes
:2
Identity_32?
AssignVariableOp_32AssignVariableOp(assignvariableop_32_adam_dense_15_bias_vIdentity_32:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_329
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOp?
Identity_33Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2
Identity_33?
Identity_34IdentityIdentity_33:output:0^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*
T0*
_output_shapes
: 2
Identity_34"#
identity_34Identity_34:output:0*?
_input_shapes?
?: :::::::::::::::::::::::::::::::::2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132*
AssignVariableOp_14AssignVariableOp_142*
AssignVariableOp_15AssignVariableOp_152*
AssignVariableOp_16AssignVariableOp_162*
AssignVariableOp_17AssignVariableOp_172*
AssignVariableOp_18AssignVariableOp_182*
AssignVariableOp_19AssignVariableOp_192(
AssignVariableOp_2AssignVariableOp_22*
AssignVariableOp_20AssignVariableOp_202*
AssignVariableOp_21AssignVariableOp_212*
AssignVariableOp_22AssignVariableOp_222*
AssignVariableOp_23AssignVariableOp_232*
AssignVariableOp_24AssignVariableOp_242*
AssignVariableOp_25AssignVariableOp_252*
AssignVariableOp_26AssignVariableOp_262*
AssignVariableOp_27AssignVariableOp_272*
AssignVariableOp_28AssignVariableOp_282*
AssignVariableOp_29AssignVariableOp_292(
AssignVariableOp_3AssignVariableOp_32*
AssignVariableOp_30AssignVariableOp_302*
AssignVariableOp_31AssignVariableOp_312*
AssignVariableOp_32AssignVariableOp_322(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_9:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
?
c
G__inference_activation_25_layer_call_and_return_conditional_losses_8706

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????bb 2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????bb :W S
/
_output_shapes
:?????????bb 
 
_user_specified_nameinputs
?5
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_9018

inputs
conv2d_24_8987
conv2d_24_8989
conv2d_25_8994
conv2d_25_8996
conv2d_26_9001
conv2d_26_9003
dense_15_9011
dense_15_9013
identity??!conv2d_24/StatefulPartitionedCall?!conv2d_25/StatefulPartitionedCall?!conv2d_26/StatefulPartitionedCall? dense_15/StatefulPartitionedCall?
!conv2d_24/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_24_8987conv2d_24_8989*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_24_layer_call_and_return_conditional_losses_86852#
!conv2d_24/StatefulPartitionedCall?
activation_25/PartitionedCallPartitionedCall*conv2d_24/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_25_layer_call_and_return_conditional_losses_87062
activation_25/PartitionedCall?
 max_pooling2d_24/PartitionedCallPartitionedCall&activation_25/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????11 * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_86412"
 max_pooling2d_24/PartitionedCall?
!conv2d_25/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_24/PartitionedCall:output:0conv2d_25_8994conv2d_25_8996*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_25_layer_call_and_return_conditional_losses_87252#
!conv2d_25/StatefulPartitionedCall?
activation_26/PartitionedCallPartitionedCall*conv2d_25/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_26_layer_call_and_return_conditional_losses_87462
activation_26/PartitionedCall?
 max_pooling2d_25/PartitionedCallPartitionedCall&activation_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:????????? * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_86532"
 max_pooling2d_25/PartitionedCall?
!conv2d_26/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_25/PartitionedCall:output:0conv2d_26_9001conv2d_26_9003*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_26_layer_call_and_return_conditional_losses_87652#
!conv2d_26/StatefulPartitionedCall?
activation_27/PartitionedCallPartitionedCall*conv2d_26/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_27_layer_call_and_return_conditional_losses_87862
activation_27/PartitionedCall?
 max_pooling2d_26/PartitionedCallPartitionedCall&activation_27/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????

@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_86652"
 max_pooling2d_26/PartitionedCall?
flatten_8/PartitionedCallPartitionedCall)max_pooling2d_26/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_flatten_8_layer_call_and_return_conditional_losses_88012
flatten_8/PartitionedCall?
activation_28/PartitionedCallPartitionedCall"flatten_8/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_28_layer_call_and_return_conditional_losses_88142
activation_28/PartitionedCall?
dropout_5/PartitionedCallPartitionedCall&activation_28/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88392
dropout_5/PartitionedCall?
 dense_15/StatefulPartitionedCallStatefulPartitionedCall"dropout_5/PartitionedCall:output:0dense_15_9011dense_15_9013*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *K
fFRD
B__inference_dense_15_layer_call_and_return_conditional_losses_88622"
 dense_15/StatefulPartitionedCall?
activation_29/PartitionedCallPartitionedCall)dense_15/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_29_layer_call_and_return_conditional_losses_88832
activation_29/PartitionedCall?
IdentityIdentity&activation_29/PartitionedCall:output:0"^conv2d_24/StatefulPartitionedCall"^conv2d_25/StatefulPartitionedCall"^conv2d_26/StatefulPartitionedCall!^dense_15/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2F
!conv2d_24/StatefulPartitionedCall!conv2d_24/StatefulPartitionedCall2F
!conv2d_25/StatefulPartitionedCall!conv2d_25/StatefulPartitionedCall2F
!conv2d_26/StatefulPartitionedCall!conv2d_26/StatefulPartitionedCall2D
 dense_15/StatefulPartitionedCall dense_15/StatefulPartitionedCall:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
K
/__inference_max_pooling2d_24_layer_call_fn_8647

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *J
_output_shapes8
6:4????????????????????????????????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_86412
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?	
?
B__inference_dense_15_layer_call_and_return_conditional_losses_9340

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	?2*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????2::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
c
G__inference_activation_26_layer_call_and_return_conditional_losses_9248

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????// 2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????// :W S
/
_output_shapes
:?????????// 
 
_user_specified_nameinputs
?
b
C__inference_dropout_5_layer_call_and_return_conditional_losses_8834

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????22
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????2*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????22
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????22
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????22
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
f
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_8653

inputs
identity?
MaxPoolMaxPoolinputs*J
_output_shapes8
6:4????????????????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
IdentityIdentityMaxPool:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
K
/__inference_max_pooling2d_25_layer_call_fn_8659

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *J
_output_shapes8
6:4????????????????????????????????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_86532
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
K
/__inference_max_pooling2d_26_layer_call_fn_8671

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *J
_output_shapes8
6:4????????????????????????????????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *S
fNRL
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_86652
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
H
,__inference_activation_26_layer_call_fn_9253

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_26_layer_call_and_return_conditional_losses_87462
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????// :W S
/
_output_shapes
:?????????// 
 
_user_specified_nameinputs
?
D
(__inference_flatten_8_layer_call_fn_9293

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_flatten_8_layer_call_and_return_conditional_losses_88012
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????

@:W S
/
_output_shapes
:?????????

@
 
_user_specified_nameinputs
?
?
+__inference_sequential_8_layer_call_fn_9195

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_sequential_8_layer_call_and_return_conditional_losses_90182
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?I
?
__inference__traced_save_9481
file_prefix/
+savev2_conv2d_24_kernel_read_readvariableop-
)savev2_conv2d_24_bias_read_readvariableop/
+savev2_conv2d_25_kernel_read_readvariableop-
)savev2_conv2d_25_bias_read_readvariableop/
+savev2_conv2d_26_kernel_read_readvariableop-
)savev2_conv2d_26_bias_read_readvariableop.
*savev2_dense_15_kernel_read_readvariableop,
(savev2_dense_15_bias_read_readvariableop(
$savev2_adam_iter_read_readvariableop	*
&savev2_adam_beta_1_read_readvariableop*
&savev2_adam_beta_2_read_readvariableop)
%savev2_adam_decay_read_readvariableop1
-savev2_adam_learning_rate_read_readvariableop$
 savev2_total_read_readvariableop$
 savev2_count_read_readvariableop&
"savev2_total_1_read_readvariableop&
"savev2_count_1_read_readvariableop6
2savev2_adam_conv2d_24_kernel_m_read_readvariableop4
0savev2_adam_conv2d_24_bias_m_read_readvariableop6
2savev2_adam_conv2d_25_kernel_m_read_readvariableop4
0savev2_adam_conv2d_25_bias_m_read_readvariableop6
2savev2_adam_conv2d_26_kernel_m_read_readvariableop4
0savev2_adam_conv2d_26_bias_m_read_readvariableop5
1savev2_adam_dense_15_kernel_m_read_readvariableop3
/savev2_adam_dense_15_bias_m_read_readvariableop6
2savev2_adam_conv2d_24_kernel_v_read_readvariableop4
0savev2_adam_conv2d_24_bias_v_read_readvariableop6
2savev2_adam_conv2d_25_kernel_v_read_readvariableop4
0savev2_adam_conv2d_25_bias_v_read_readvariableop6
2savev2_adam_conv2d_26_kernel_v_read_readvariableop4
0savev2_adam_conv2d_26_bias_v_read_readvariableop5
1savev2_adam_dense_15_kernel_v_read_readvariableop3
/savev2_adam_dense_15_bias_v_read_readvariableop
savev2_const

identity_1??MergeV2Checkpoints?
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
Const_1?
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shard?
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilename?
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:"*
dtype0*?
value?B?"B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-3/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-3/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names?
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:"*
dtype0*W
valueNBL"B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slices?
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0+savev2_conv2d_24_kernel_read_readvariableop)savev2_conv2d_24_bias_read_readvariableop+savev2_conv2d_25_kernel_read_readvariableop)savev2_conv2d_25_bias_read_readvariableop+savev2_conv2d_26_kernel_read_readvariableop)savev2_conv2d_26_bias_read_readvariableop*savev2_dense_15_kernel_read_readvariableop(savev2_dense_15_bias_read_readvariableop$savev2_adam_iter_read_readvariableop&savev2_adam_beta_1_read_readvariableop&savev2_adam_beta_2_read_readvariableop%savev2_adam_decay_read_readvariableop-savev2_adam_learning_rate_read_readvariableop savev2_total_read_readvariableop savev2_count_read_readvariableop"savev2_total_1_read_readvariableop"savev2_count_1_read_readvariableop2savev2_adam_conv2d_24_kernel_m_read_readvariableop0savev2_adam_conv2d_24_bias_m_read_readvariableop2savev2_adam_conv2d_25_kernel_m_read_readvariableop0savev2_adam_conv2d_25_bias_m_read_readvariableop2savev2_adam_conv2d_26_kernel_m_read_readvariableop0savev2_adam_conv2d_26_bias_m_read_readvariableop1savev2_adam_dense_15_kernel_m_read_readvariableop/savev2_adam_dense_15_bias_m_read_readvariableop2savev2_adam_conv2d_24_kernel_v_read_readvariableop0savev2_adam_conv2d_24_bias_v_read_readvariableop2savev2_adam_conv2d_25_kernel_v_read_readvariableop0savev2_adam_conv2d_25_bias_v_read_readvariableop2savev2_adam_conv2d_26_kernel_v_read_readvariableop0savev2_adam_conv2d_26_bias_v_read_readvariableop1savev2_adam_dense_15_kernel_v_read_readvariableop/savev2_adam_dense_15_bias_v_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *0
dtypes&
$2"	2
SaveV2?
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes?
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identitym

Identity_1IdentityIdentity:output:0^MergeV2Checkpoints*
T0*
_output_shapes
: 2

Identity_1"!

identity_1Identity_1:output:0*?
_input_shapes?
?: : : :  : : @:@:	?2:: : : : : : : : : : : :  : : @:@:	?2:: : :  : : @:@:	?2:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:,(
&
_output_shapes
: : 

_output_shapes
: :,(
&
_output_shapes
:  : 

_output_shapes
: :,(
&
_output_shapes
: @: 

_output_shapes
:@:%!

_output_shapes
:	?2: 

_output_shapes
::	

_output_shapes
: :


_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :,(
&
_output_shapes
: : 

_output_shapes
: :,(
&
_output_shapes
:  : 

_output_shapes
: :,(
&
_output_shapes
: @: 

_output_shapes
:@:%!

_output_shapes
:	?2: 

_output_shapes
::,(
&
_output_shapes
: : 

_output_shapes
: :,(
&
_output_shapes
:  : 

_output_shapes
: :,(
&
_output_shapes
: @: 

_output_shapes
:@:% !

_output_shapes
:	?2: !

_output_shapes
::"

_output_shapes
: 
?	
?
C__inference_conv2d_25_layer_call_and_return_conditional_losses_9234

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:  *
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// *
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
: *
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// 2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????11 ::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????11 
 
_user_specified_nameinputs
?
D
(__inference_dropout_5_layer_call_fn_9330

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_dropout_5_layer_call_and_return_conditional_losses_88392
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
c
G__inference_activation_27_layer_call_and_return_conditional_losses_8786

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????@2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????@:W S
/
_output_shapes
:?????????@
 
_user_specified_nameinputs
?@
?
__inference__wrapped_model_8635
conv2d_24_input9
5sequential_8_conv2d_24_conv2d_readvariableop_resource:
6sequential_8_conv2d_24_biasadd_readvariableop_resource9
5sequential_8_conv2d_25_conv2d_readvariableop_resource:
6sequential_8_conv2d_25_biasadd_readvariableop_resource9
5sequential_8_conv2d_26_conv2d_readvariableop_resource:
6sequential_8_conv2d_26_biasadd_readvariableop_resource8
4sequential_8_dense_15_matmul_readvariableop_resource9
5sequential_8_dense_15_biasadd_readvariableop_resource
identity??-sequential_8/conv2d_24/BiasAdd/ReadVariableOp?,sequential_8/conv2d_24/Conv2D/ReadVariableOp?-sequential_8/conv2d_25/BiasAdd/ReadVariableOp?,sequential_8/conv2d_25/Conv2D/ReadVariableOp?-sequential_8/conv2d_26/BiasAdd/ReadVariableOp?,sequential_8/conv2d_26/Conv2D/ReadVariableOp?,sequential_8/dense_15/BiasAdd/ReadVariableOp?+sequential_8/dense_15/MatMul/ReadVariableOp?
,sequential_8/conv2d_24/Conv2D/ReadVariableOpReadVariableOp5sequential_8_conv2d_24_conv2d_readvariableop_resource*&
_output_shapes
: *
dtype02.
,sequential_8/conv2d_24/Conv2D/ReadVariableOp?
sequential_8/conv2d_24/Conv2DConv2Dconv2d_24_input4sequential_8/conv2d_24/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb *
paddingVALID*
strides
2
sequential_8/conv2d_24/Conv2D?
-sequential_8/conv2d_24/BiasAdd/ReadVariableOpReadVariableOp6sequential_8_conv2d_24_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02/
-sequential_8/conv2d_24/BiasAdd/ReadVariableOp?
sequential_8/conv2d_24/BiasAddBiasAdd&sequential_8/conv2d_24/Conv2D:output:05sequential_8/conv2d_24/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb 2 
sequential_8/conv2d_24/BiasAdd?
sequential_8/activation_25/ReluRelu'sequential_8/conv2d_24/BiasAdd:output:0*
T0*/
_output_shapes
:?????????bb 2!
sequential_8/activation_25/Relu?
%sequential_8/max_pooling2d_24/MaxPoolMaxPool-sequential_8/activation_25/Relu:activations:0*/
_output_shapes
:?????????11 *
ksize
*
paddingVALID*
strides
2'
%sequential_8/max_pooling2d_24/MaxPool?
,sequential_8/conv2d_25/Conv2D/ReadVariableOpReadVariableOp5sequential_8_conv2d_25_conv2d_readvariableop_resource*&
_output_shapes
:  *
dtype02.
,sequential_8/conv2d_25/Conv2D/ReadVariableOp?
sequential_8/conv2d_25/Conv2DConv2D.sequential_8/max_pooling2d_24/MaxPool:output:04sequential_8/conv2d_25/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// *
paddingVALID*
strides
2
sequential_8/conv2d_25/Conv2D?
-sequential_8/conv2d_25/BiasAdd/ReadVariableOpReadVariableOp6sequential_8_conv2d_25_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02/
-sequential_8/conv2d_25/BiasAdd/ReadVariableOp?
sequential_8/conv2d_25/BiasAddBiasAdd&sequential_8/conv2d_25/Conv2D:output:05sequential_8/conv2d_25/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// 2 
sequential_8/conv2d_25/BiasAdd?
sequential_8/activation_26/ReluRelu'sequential_8/conv2d_25/BiasAdd:output:0*
T0*/
_output_shapes
:?????????// 2!
sequential_8/activation_26/Relu?
%sequential_8/max_pooling2d_25/MaxPoolMaxPool-sequential_8/activation_26/Relu:activations:0*/
_output_shapes
:????????? *
ksize
*
paddingVALID*
strides
2'
%sequential_8/max_pooling2d_25/MaxPool?
,sequential_8/conv2d_26/Conv2D/ReadVariableOpReadVariableOp5sequential_8_conv2d_26_conv2d_readvariableop_resource*&
_output_shapes
: @*
dtype02.
,sequential_8/conv2d_26/Conv2D/ReadVariableOp?
sequential_8/conv2d_26/Conv2DConv2D.sequential_8/max_pooling2d_25/MaxPool:output:04sequential_8/conv2d_26/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@*
paddingVALID*
strides
2
sequential_8/conv2d_26/Conv2D?
-sequential_8/conv2d_26/BiasAdd/ReadVariableOpReadVariableOp6sequential_8_conv2d_26_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype02/
-sequential_8/conv2d_26/BiasAdd/ReadVariableOp?
sequential_8/conv2d_26/BiasAddBiasAdd&sequential_8/conv2d_26/Conv2D:output:05sequential_8/conv2d_26/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@2 
sequential_8/conv2d_26/BiasAdd?
sequential_8/activation_27/ReluRelu'sequential_8/conv2d_26/BiasAdd:output:0*
T0*/
_output_shapes
:?????????@2!
sequential_8/activation_27/Relu?
%sequential_8/max_pooling2d_26/MaxPoolMaxPool-sequential_8/activation_27/Relu:activations:0*/
_output_shapes
:?????????

@*
ksize
*
paddingVALID*
strides
2'
%sequential_8/max_pooling2d_26/MaxPool?
sequential_8/flatten_8/ConstConst*
_output_shapes
:*
dtype0*
valueB"????   2
sequential_8/flatten_8/Const?
sequential_8/flatten_8/ReshapeReshape.sequential_8/max_pooling2d_26/MaxPool:output:0%sequential_8/flatten_8/Const:output:0*
T0*(
_output_shapes
:??????????22 
sequential_8/flatten_8/Reshape?
sequential_8/activation_28/ReluRelu'sequential_8/flatten_8/Reshape:output:0*
T0*(
_output_shapes
:??????????22!
sequential_8/activation_28/Relu?
sequential_8/dropout_5/IdentityIdentity-sequential_8/activation_28/Relu:activations:0*
T0*(
_output_shapes
:??????????22!
sequential_8/dropout_5/Identity?
+sequential_8/dense_15/MatMul/ReadVariableOpReadVariableOp4sequential_8_dense_15_matmul_readvariableop_resource*
_output_shapes
:	?2*
dtype02-
+sequential_8/dense_15/MatMul/ReadVariableOp?
sequential_8/dense_15/MatMulMatMul(sequential_8/dropout_5/Identity:output:03sequential_8/dense_15/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
sequential_8/dense_15/MatMul?
,sequential_8/dense_15/BiasAdd/ReadVariableOpReadVariableOp5sequential_8_dense_15_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02.
,sequential_8/dense_15/BiasAdd/ReadVariableOp?
sequential_8/dense_15/BiasAddBiasAdd&sequential_8/dense_15/MatMul:product:04sequential_8/dense_15/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
sequential_8/dense_15/BiasAdd?
"sequential_8/activation_29/SigmoidSigmoid&sequential_8/dense_15/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2$
"sequential_8/activation_29/Sigmoid?
IdentityIdentity&sequential_8/activation_29/Sigmoid:y:0.^sequential_8/conv2d_24/BiasAdd/ReadVariableOp-^sequential_8/conv2d_24/Conv2D/ReadVariableOp.^sequential_8/conv2d_25/BiasAdd/ReadVariableOp-^sequential_8/conv2d_25/Conv2D/ReadVariableOp.^sequential_8/conv2d_26/BiasAdd/ReadVariableOp-^sequential_8/conv2d_26/Conv2D/ReadVariableOp-^sequential_8/dense_15/BiasAdd/ReadVariableOp,^sequential_8/dense_15/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2^
-sequential_8/conv2d_24/BiasAdd/ReadVariableOp-sequential_8/conv2d_24/BiasAdd/ReadVariableOp2\
,sequential_8/conv2d_24/Conv2D/ReadVariableOp,sequential_8/conv2d_24/Conv2D/ReadVariableOp2^
-sequential_8/conv2d_25/BiasAdd/ReadVariableOp-sequential_8/conv2d_25/BiasAdd/ReadVariableOp2\
,sequential_8/conv2d_25/Conv2D/ReadVariableOp,sequential_8/conv2d_25/Conv2D/ReadVariableOp2^
-sequential_8/conv2d_26/BiasAdd/ReadVariableOp-sequential_8/conv2d_26/BiasAdd/ReadVariableOp2\
,sequential_8/conv2d_26/Conv2D/ReadVariableOp,sequential_8/conv2d_26/Conv2D/ReadVariableOp2\
,sequential_8/dense_15/BiasAdd/ReadVariableOp,sequential_8/dense_15/BiasAdd/ReadVariableOp2Z
+sequential_8/dense_15/MatMul/ReadVariableOp+sequential_8/dense_15/MatMul/ReadVariableOp:` \
/
_output_shapes
:?????????dd
)
_user_specified_nameconv2d_24_input
?
f
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_8665

inputs
identity?
MaxPoolMaxPoolinputs*J
_output_shapes8
6:4????????????????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
IdentityIdentityMaxPool:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
b
C__inference_dropout_5_layer_call_and_return_conditional_losses_9315

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????22
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????2*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????22
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????22
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????22
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
c
G__inference_activation_26_layer_call_and_return_conditional_losses_8746

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????// 2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????// :W S
/
_output_shapes
:?????????// 
 
_user_specified_nameinputs
?
c
G__inference_activation_28_layer_call_and_return_conditional_losses_8814

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????22
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
}
(__inference_conv2d_25_layer_call_fn_9243

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????// *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_25_layer_call_and_return_conditional_losses_87252
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????// 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????11 ::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????11 
 
_user_specified_nameinputs
?	
?
C__inference_conv2d_24_layer_call_and_return_conditional_losses_8685

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
: *
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb *
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
: *
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb 2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????dd::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
H
,__inference_activation_27_layer_call_fn_9282

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_27_layer_call_and_return_conditional_losses_87862
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????@:W S
/
_output_shapes
:?????????@
 
_user_specified_nameinputs
?
H
,__inference_activation_28_layer_call_fn_9303

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????2* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *P
fKRI
G__inference_activation_28_layer_call_and_return_conditional_losses_88142
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????2:P L
(
_output_shapes
:??????????2
 
_user_specified_nameinputs
?
}
(__inference_conv2d_24_layer_call_fn_9214

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????bb *$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_24_layer_call_and_return_conditional_losses_86852
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????bb 2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????dd::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
?
+__inference_sequential_8_layer_call_fn_9174

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6*
Tin
2	*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????**
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_sequential_8_layer_call_and_return_conditional_losses_89632
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?
}
(__inference_conv2d_26_layer_call_fn_9272

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????@*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *L
fGRE
C__inference_conv2d_26_layer_call_and_return_conditional_losses_87652
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:????????? ::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:????????? 
 
_user_specified_nameinputs
?
_
C__inference_flatten_8_layer_call_and_return_conditional_losses_9288

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"????   2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????22	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????22

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????

@:W S
/
_output_shapes
:?????????

@
 
_user_specified_nameinputs
?3
?
F__inference_sequential_8_layer_call_and_return_conditional_losses_9153

inputs,
(conv2d_24_conv2d_readvariableop_resource-
)conv2d_24_biasadd_readvariableop_resource,
(conv2d_25_conv2d_readvariableop_resource-
)conv2d_25_biasadd_readvariableop_resource,
(conv2d_26_conv2d_readvariableop_resource-
)conv2d_26_biasadd_readvariableop_resource+
'dense_15_matmul_readvariableop_resource,
(dense_15_biasadd_readvariableop_resource
identity?? conv2d_24/BiasAdd/ReadVariableOp?conv2d_24/Conv2D/ReadVariableOp? conv2d_25/BiasAdd/ReadVariableOp?conv2d_25/Conv2D/ReadVariableOp? conv2d_26/BiasAdd/ReadVariableOp?conv2d_26/Conv2D/ReadVariableOp?dense_15/BiasAdd/ReadVariableOp?dense_15/MatMul/ReadVariableOp?
conv2d_24/Conv2D/ReadVariableOpReadVariableOp(conv2d_24_conv2d_readvariableop_resource*&
_output_shapes
: *
dtype02!
conv2d_24/Conv2D/ReadVariableOp?
conv2d_24/Conv2DConv2Dinputs'conv2d_24/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb *
paddingVALID*
strides
2
conv2d_24/Conv2D?
 conv2d_24/BiasAdd/ReadVariableOpReadVariableOp)conv2d_24_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02"
 conv2d_24/BiasAdd/ReadVariableOp?
conv2d_24/BiasAddBiasAddconv2d_24/Conv2D:output:0(conv2d_24/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????bb 2
conv2d_24/BiasAdd?
activation_25/ReluReluconv2d_24/BiasAdd:output:0*
T0*/
_output_shapes
:?????????bb 2
activation_25/Relu?
max_pooling2d_24/MaxPoolMaxPool activation_25/Relu:activations:0*/
_output_shapes
:?????????11 *
ksize
*
paddingVALID*
strides
2
max_pooling2d_24/MaxPool?
conv2d_25/Conv2D/ReadVariableOpReadVariableOp(conv2d_25_conv2d_readvariableop_resource*&
_output_shapes
:  *
dtype02!
conv2d_25/Conv2D/ReadVariableOp?
conv2d_25/Conv2DConv2D!max_pooling2d_24/MaxPool:output:0'conv2d_25/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// *
paddingVALID*
strides
2
conv2d_25/Conv2D?
 conv2d_25/BiasAdd/ReadVariableOpReadVariableOp)conv2d_25_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02"
 conv2d_25/BiasAdd/ReadVariableOp?
conv2d_25/BiasAddBiasAddconv2d_25/Conv2D:output:0(conv2d_25/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????// 2
conv2d_25/BiasAdd?
activation_26/ReluReluconv2d_25/BiasAdd:output:0*
T0*/
_output_shapes
:?????????// 2
activation_26/Relu?
max_pooling2d_25/MaxPoolMaxPool activation_26/Relu:activations:0*/
_output_shapes
:????????? *
ksize
*
paddingVALID*
strides
2
max_pooling2d_25/MaxPool?
conv2d_26/Conv2D/ReadVariableOpReadVariableOp(conv2d_26_conv2d_readvariableop_resource*&
_output_shapes
: @*
dtype02!
conv2d_26/Conv2D/ReadVariableOp?
conv2d_26/Conv2DConv2D!max_pooling2d_25/MaxPool:output:0'conv2d_26/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@*
paddingVALID*
strides
2
conv2d_26/Conv2D?
 conv2d_26/BiasAdd/ReadVariableOpReadVariableOp)conv2d_26_biasadd_readvariableop_resource*
_output_shapes
:@*
dtype02"
 conv2d_26/BiasAdd/ReadVariableOp?
conv2d_26/BiasAddBiasAddconv2d_26/Conv2D:output:0(conv2d_26/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@2
conv2d_26/BiasAdd?
activation_27/ReluReluconv2d_26/BiasAdd:output:0*
T0*/
_output_shapes
:?????????@2
activation_27/Relu?
max_pooling2d_26/MaxPoolMaxPool activation_27/Relu:activations:0*/
_output_shapes
:?????????

@*
ksize
*
paddingVALID*
strides
2
max_pooling2d_26/MaxPools
flatten_8/ConstConst*
_output_shapes
:*
dtype0*
valueB"????   2
flatten_8/Const?
flatten_8/ReshapeReshape!max_pooling2d_26/MaxPool:output:0flatten_8/Const:output:0*
T0*(
_output_shapes
:??????????22
flatten_8/Reshape
activation_28/ReluReluflatten_8/Reshape:output:0*
T0*(
_output_shapes
:??????????22
activation_28/Relu?
dropout_5/IdentityIdentity activation_28/Relu:activations:0*
T0*(
_output_shapes
:??????????22
dropout_5/Identity?
dense_15/MatMul/ReadVariableOpReadVariableOp'dense_15_matmul_readvariableop_resource*
_output_shapes
:	?2*
dtype02 
dense_15/MatMul/ReadVariableOp?
dense_15/MatMulMatMuldropout_5/Identity:output:0&dense_15/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_15/MatMul?
dense_15/BiasAdd/ReadVariableOpReadVariableOp(dense_15_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_15/BiasAdd/ReadVariableOp?
dense_15/BiasAddBiasAdddense_15/MatMul:product:0'dense_15/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_15/BiasAdd?
activation_29/SigmoidSigmoiddense_15/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_29/Sigmoid?
IdentityIdentityactivation_29/Sigmoid:y:0!^conv2d_24/BiasAdd/ReadVariableOp ^conv2d_24/Conv2D/ReadVariableOp!^conv2d_25/BiasAdd/ReadVariableOp ^conv2d_25/Conv2D/ReadVariableOp!^conv2d_26/BiasAdd/ReadVariableOp ^conv2d_26/Conv2D/ReadVariableOp ^dense_15/BiasAdd/ReadVariableOp^dense_15/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*N
_input_shapes=
;:?????????dd::::::::2D
 conv2d_24/BiasAdd/ReadVariableOp conv2d_24/BiasAdd/ReadVariableOp2B
conv2d_24/Conv2D/ReadVariableOpconv2d_24/Conv2D/ReadVariableOp2D
 conv2d_25/BiasAdd/ReadVariableOp conv2d_25/BiasAdd/ReadVariableOp2B
conv2d_25/Conv2D/ReadVariableOpconv2d_25/Conv2D/ReadVariableOp2D
 conv2d_26/BiasAdd/ReadVariableOp conv2d_26/BiasAdd/ReadVariableOp2B
conv2d_26/Conv2D/ReadVariableOpconv2d_26/Conv2D/ReadVariableOp2B
dense_15/BiasAdd/ReadVariableOpdense_15/BiasAdd/ReadVariableOp2@
dense_15/MatMul/ReadVariableOpdense_15/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????dd
 
_user_specified_nameinputs
?	
?
C__inference_conv2d_26_layer_call_and_return_conditional_losses_8765

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
: @*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:@*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????@2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:????????? ::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:????????? 
 
_user_specified_nameinputs
?
c
G__inference_activation_27_layer_call_and_return_conditional_losses_9277

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????@2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????@2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????@:W S
/
_output_shapes
:?????????@
 
_user_specified_nameinputs"?L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*?
serving_default?
S
conv2d_24_input@
!serving_default_conv2d_24_input:0?????????ddA
activation_290
StatefulPartitionedCall:0?????????tensorflow/serving/predict:??
?S
layer_with_weights-0
layer-0
layer-1
layer-2
layer_with_weights-1
layer-3
layer-4
layer-5
layer_with_weights-2
layer-6
layer-7
	layer-8

layer-9
layer-10
layer-11
layer_with_weights-3
layer-12
layer-13
	optimizer
	variables
regularization_losses
trainable_variables
	keras_api

signatures
?__call__
?_default_save_signature
+?&call_and_return_all_conditional_losses"?P
_tf_keras_sequential?O{"class_name": "Sequential", "name": "sequential_8", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "config": {"name": "sequential_8", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_24_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_24", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_25", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_24", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_25", "trainable": true, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_26", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_25", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_26", "trainable": true, "dtype": "float32", "filters": 64, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_27", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_26", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Flatten", "config": {"name": "flatten_8", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Activation", "config": {"name": "activation_28", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "Dropout", "config": {"name": "dropout_5", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_15", "trainable": true, "dtype": "float32", "units": 3, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_29", "trainable": true, "dtype": "float32", "activation": "sigmoid"}}]}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 100, 100, 3]}, "is_graph_network": true, "keras_version": "2.4.0", "backend": "tensorflow", "model_config": {"class_name": "Sequential", "config": {"name": "sequential_8", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_24_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_24", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_25", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_24", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_25", "trainable": true, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_26", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_25", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_26", "trainable": true, "dtype": "float32", "filters": 64, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_27", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_26", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Flatten", "config": {"name": "flatten_8", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Activation", "config": {"name": "activation_28", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "Dropout", "config": {"name": "dropout_5", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_15", "trainable": true, "dtype": "float32", "units": 3, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_29", "trainable": true, "dtype": "float32", "activation": "sigmoid"}}]}}, "training_config": {"loss": {"class_name": "SparseCategoricalCrossentropy", "config": {"reduction": "auto", "name": "sparse_categorical_crossentropy", "from_logits": true}}, "metrics": [[{"class_name": "MeanMetricWrapper", "config": {"name": "accuracy", "dtype": "float32", "fn": "sparse_categorical_accuracy"}}]], "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Adam", "config": {"name": "Adam", "learning_rate": 0.0010000000474974513, "decay": 0.0, "beta_1": 0.8999999761581421, "beta_2": 0.9990000128746033, "epsilon": 1e-07, "amsgrad": false}}}}
?


kernel
bias
	variables
regularization_losses
trainable_variables
	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?	
_tf_keras_layer?	{"class_name": "Conv2D", "name": "conv2d_24", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_24", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 100, 100, 3]}, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 100, 100, 3]}}
?
	variables
regularization_losses
trainable_variables
	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_25", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_25", "trainable": true, "dtype": "float32", "activation": "relu"}}
?
	variables
 regularization_losses
!trainable_variables
"	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_24", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_24", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?	

#kernel
$bias
%	variables
&regularization_losses
'trainable_variables
(	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_25", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_25", "trainable": true, "dtype": "float32", "filters": 32, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 32}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 49, 49, 32]}}
?
)	variables
*regularization_losses
+trainable_variables
,	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_26", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_26", "trainable": true, "dtype": "float32", "activation": "relu"}}
?
-	variables
.regularization_losses
/trainable_variables
0	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_25", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_25", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?	

1kernel
2bias
3	variables
4regularization_losses
5trainable_variables
6	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_26", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_26", "trainable": true, "dtype": "float32", "filters": 64, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 32}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 23, 23, 32]}}
?
7	variables
8regularization_losses
9trainable_variables
:	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_27", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_27", "trainable": true, "dtype": "float32", "activation": "relu"}}
?
;	variables
<regularization_losses
=trainable_variables
>	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_26", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_26", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?
?	variables
@regularization_losses
Atrainable_variables
B	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Flatten", "name": "flatten_8", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "flatten_8", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}}}
?
C	variables
Dregularization_losses
Etrainable_variables
F	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_28", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_28", "trainable": true, "dtype": "float32", "activation": "relu"}}
?
G	variables
Hregularization_losses
Itrainable_variables
J	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_5", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_5", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}}
?

Kkernel
Lbias
M	variables
Nregularization_losses
Otrainable_variables
P	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_15", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_15", "trainable": true, "dtype": "float32", "units": 3, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 6400}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 6400]}}
?
Q	variables
Rregularization_losses
Strainable_variables
T	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_29", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_29", "trainable": true, "dtype": "float32", "activation": "sigmoid"}}
?
Uiter

Vbeta_1

Wbeta_2
	Xdecay
Ylearning_ratem?m?#m?$m?1m?2m?Km?Lm?v?v?#v?$v?1v?2v?Kv?Lv?"
	optimizer
X
0
1
#2
$3
14
25
K6
L7"
trackable_list_wrapper
 "
trackable_list_wrapper
X
0
1
#2
$3
14
25
K6
L7"
trackable_list_wrapper
?
	variables
Zlayer_metrics
[layer_regularization_losses
\metrics
regularization_losses

]layers
trainable_variables
^non_trainable_variables
?__call__
?_default_save_signature
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
-
?serving_default"
signature_map
*:( 2conv2d_24/kernel
: 2conv2d_24/bias
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
?
	variables
_layer_metrics
`layer_regularization_losses
ametrics
regularization_losses

blayers
trainable_variables
cnon_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
	variables
dlayer_metrics
elayer_regularization_losses
fmetrics
regularization_losses

glayers
trainable_variables
hnon_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
	variables
ilayer_metrics
jlayer_regularization_losses
kmetrics
 regularization_losses

llayers
!trainable_variables
mnon_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
*:(  2conv2d_25/kernel
: 2conv2d_25/bias
.
#0
$1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
#0
$1"
trackable_list_wrapper
?
%	variables
nlayer_metrics
olayer_regularization_losses
pmetrics
&regularization_losses

qlayers
'trainable_variables
rnon_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
)	variables
slayer_metrics
tlayer_regularization_losses
umetrics
*regularization_losses

vlayers
+trainable_variables
wnon_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
-	variables
xlayer_metrics
ylayer_regularization_losses
zmetrics
.regularization_losses

{layers
/trainable_variables
|non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
*:( @2conv2d_26/kernel
:@2conv2d_26/bias
.
10
21"
trackable_list_wrapper
 "
trackable_list_wrapper
.
10
21"
trackable_list_wrapper
?
3	variables
}layer_metrics
~layer_regularization_losses
metrics
4regularization_losses
?layers
5trainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
7	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
8regularization_losses
?layers
9trainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
;	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
<regularization_losses
?layers
=trainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
@regularization_losses
?layers
Atrainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
C	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Dregularization_losses
?layers
Etrainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
G	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Hregularization_losses
?layers
Itrainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
": 	?22dense_15/kernel
:2dense_15/bias
.
K0
L1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
K0
L1"
trackable_list_wrapper
?
M	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Nregularization_losses
?layers
Otrainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
Q	variables
?layer_metrics
 ?layer_regularization_losses
?metrics
Rregularization_losses
?layers
Strainable_variables
?non_trainable_variables
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
:	 (2	Adam/iter
: (2Adam/beta_1
: (2Adam/beta_2
: (2
Adam/decay
: (2Adam/learning_rate
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
?
0
1
2
3
4
5
6
7
	8

9
10
11
12
13"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?

?total

?count
?	variables
?	keras_api"?
_tf_keras_metricj{"class_name": "Mean", "name": "loss", "dtype": "float32", "config": {"name": "loss", "dtype": "float32"}}
?

?total

?count
?
_fn_kwargs
?	variables
?	keras_api"?
_tf_keras_metric?{"class_name": "MeanMetricWrapper", "name": "accuracy", "dtype": "float32", "config": {"name": "accuracy", "dtype": "float32", "fn": "sparse_categorical_accuracy"}}
:  (2total
:  (2count
0
?0
?1"
trackable_list_wrapper
.
?	variables"
_generic_user_object
:  (2total
:  (2count
 "
trackable_dict_wrapper
0
?0
?1"
trackable_list_wrapper
.
?	variables"
_generic_user_object
/:- 2Adam/conv2d_24/kernel/m
!: 2Adam/conv2d_24/bias/m
/:-  2Adam/conv2d_25/kernel/m
!: 2Adam/conv2d_25/bias/m
/:- @2Adam/conv2d_26/kernel/m
!:@2Adam/conv2d_26/bias/m
':%	?22Adam/dense_15/kernel/m
 :2Adam/dense_15/bias/m
/:- 2Adam/conv2d_24/kernel/v
!: 2Adam/conv2d_24/bias/v
/:-  2Adam/conv2d_25/kernel/v
!: 2Adam/conv2d_25/bias/v
/:- @2Adam/conv2d_26/kernel/v
!:@2Adam/conv2d_26/bias/v
':%	?22Adam/dense_15/kernel/v
 :2Adam/dense_15/bias/v
?2?
+__inference_sequential_8_layer_call_fn_9037
+__inference_sequential_8_layer_call_fn_8982
+__inference_sequential_8_layer_call_fn_9174
+__inference_sequential_8_layer_call_fn_9195?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
__inference__wrapped_model_8635?
???
FullArgSpec
args? 
varargsjargs
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *6?3
1?.
conv2d_24_input?????????dd
?2?
F__inference_sequential_8_layer_call_and_return_conditional_losses_8892
F__inference_sequential_8_layer_call_and_return_conditional_losses_9153
F__inference_sequential_8_layer_call_and_return_conditional_losses_9114
F__inference_sequential_8_layer_call_and_return_conditional_losses_8926?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
(__inference_conv2d_24_layer_call_fn_9214?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
C__inference_conv2d_24_layer_call_and_return_conditional_losses_9205?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
,__inference_activation_25_layer_call_fn_9224?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
G__inference_activation_25_layer_call_and_return_conditional_losses_9219?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_max_pooling2d_24_layer_call_fn_8647?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_8641?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
(__inference_conv2d_25_layer_call_fn_9243?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
C__inference_conv2d_25_layer_call_and_return_conditional_losses_9234?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
,__inference_activation_26_layer_call_fn_9253?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
G__inference_activation_26_layer_call_and_return_conditional_losses_9248?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_max_pooling2d_25_layer_call_fn_8659?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_8653?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
(__inference_conv2d_26_layer_call_fn_9272?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
C__inference_conv2d_26_layer_call_and_return_conditional_losses_9263?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
,__inference_activation_27_layer_call_fn_9282?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
G__inference_activation_27_layer_call_and_return_conditional_losses_9277?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_max_pooling2d_26_layer_call_fn_8671?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_8665?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
(__inference_flatten_8_layer_call_fn_9293?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
C__inference_flatten_8_layer_call_and_return_conditional_losses_9288?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
,__inference_activation_28_layer_call_fn_9303?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
G__inference_activation_28_layer_call_and_return_conditional_losses_9298?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
(__inference_dropout_5_layer_call_fn_9330
(__inference_dropout_5_layer_call_fn_9325?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
C__inference_dropout_5_layer_call_and_return_conditional_losses_9320
C__inference_dropout_5_layer_call_and_return_conditional_losses_9315?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
'__inference_dense_15_layer_call_fn_9349?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
B__inference_dense_15_layer_call_and_return_conditional_losses_9340?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
,__inference_activation_29_layer_call_fn_9359?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
G__inference_activation_29_layer_call_and_return_conditional_losses_9354?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?B?
"__inference_signature_wrapper_9068conv2d_24_input"?
???
FullArgSpec
args? 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 ?
__inference__wrapped_model_8635?#$12KL@?=
6?3
1?.
conv2d_24_input?????????dd
? "=?:
8
activation_29'?$
activation_29??????????
G__inference_activation_25_layer_call_and_return_conditional_losses_9219h7?4
-?*
(?%
inputs?????????bb 
? "-?*
#? 
0?????????bb 
? ?
,__inference_activation_25_layer_call_fn_9224[7?4
-?*
(?%
inputs?????????bb 
? " ??????????bb ?
G__inference_activation_26_layer_call_and_return_conditional_losses_9248h7?4
-?*
(?%
inputs?????????// 
? "-?*
#? 
0?????????// 
? ?
,__inference_activation_26_layer_call_fn_9253[7?4
-?*
(?%
inputs?????????// 
? " ??????????// ?
G__inference_activation_27_layer_call_and_return_conditional_losses_9277h7?4
-?*
(?%
inputs?????????@
? "-?*
#? 
0?????????@
? ?
,__inference_activation_27_layer_call_fn_9282[7?4
-?*
(?%
inputs?????????@
? " ??????????@?
G__inference_activation_28_layer_call_and_return_conditional_losses_9298Z0?-
&?#
!?
inputs??????????2
? "&?#
?
0??????????2
? }
,__inference_activation_28_layer_call_fn_9303M0?-
&?#
!?
inputs??????????2
? "???????????2?
G__inference_activation_29_layer_call_and_return_conditional_losses_9354X/?,
%?"
 ?
inputs?????????
? "%?"
?
0?????????
? {
,__inference_activation_29_layer_call_fn_9359K/?,
%?"
 ?
inputs?????????
? "???????????
C__inference_conv2d_24_layer_call_and_return_conditional_losses_9205l7?4
-?*
(?%
inputs?????????dd
? "-?*
#? 
0?????????bb 
? ?
(__inference_conv2d_24_layer_call_fn_9214_7?4
-?*
(?%
inputs?????????dd
? " ??????????bb ?
C__inference_conv2d_25_layer_call_and_return_conditional_losses_9234l#$7?4
-?*
(?%
inputs?????????11 
? "-?*
#? 
0?????????// 
? ?
(__inference_conv2d_25_layer_call_fn_9243_#$7?4
-?*
(?%
inputs?????????11 
? " ??????????// ?
C__inference_conv2d_26_layer_call_and_return_conditional_losses_9263l127?4
-?*
(?%
inputs????????? 
? "-?*
#? 
0?????????@
? ?
(__inference_conv2d_26_layer_call_fn_9272_127?4
-?*
(?%
inputs????????? 
? " ??????????@?
B__inference_dense_15_layer_call_and_return_conditional_losses_9340]KL0?-
&?#
!?
inputs??????????2
? "%?"
?
0?????????
? {
'__inference_dense_15_layer_call_fn_9349PKL0?-
&?#
!?
inputs??????????2
? "???????????
C__inference_dropout_5_layer_call_and_return_conditional_losses_9315^4?1
*?'
!?
inputs??????????2
p
? "&?#
?
0??????????2
? ?
C__inference_dropout_5_layer_call_and_return_conditional_losses_9320^4?1
*?'
!?
inputs??????????2
p 
? "&?#
?
0??????????2
? }
(__inference_dropout_5_layer_call_fn_9325Q4?1
*?'
!?
inputs??????????2
p
? "???????????2}
(__inference_dropout_5_layer_call_fn_9330Q4?1
*?'
!?
inputs??????????2
p 
? "???????????2?
C__inference_flatten_8_layer_call_and_return_conditional_losses_9288a7?4
-?*
(?%
inputs?????????

@
? "&?#
?
0??????????2
? ?
(__inference_flatten_8_layer_call_fn_9293T7?4
-?*
(?%
inputs?????????

@
? "???????????2?
J__inference_max_pooling2d_24_layer_call_and_return_conditional_losses_8641?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
/__inference_max_pooling2d_24_layer_call_fn_8647?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
J__inference_max_pooling2d_25_layer_call_and_return_conditional_losses_8653?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
/__inference_max_pooling2d_25_layer_call_fn_8659?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
J__inference_max_pooling2d_26_layer_call_and_return_conditional_losses_8665?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
/__inference_max_pooling2d_26_layer_call_fn_8671?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
F__inference_sequential_8_layer_call_and_return_conditional_losses_8892{#$12KLH?E
>?;
1?.
conv2d_24_input?????????dd
p

 
? "%?"
?
0?????????
? ?
F__inference_sequential_8_layer_call_and_return_conditional_losses_8926{#$12KLH?E
>?;
1?.
conv2d_24_input?????????dd
p 

 
? "%?"
?
0?????????
? ?
F__inference_sequential_8_layer_call_and_return_conditional_losses_9114r#$12KL??<
5?2
(?%
inputs?????????dd
p

 
? "%?"
?
0?????????
? ?
F__inference_sequential_8_layer_call_and_return_conditional_losses_9153r#$12KL??<
5?2
(?%
inputs?????????dd
p 

 
? "%?"
?
0?????????
? ?
+__inference_sequential_8_layer_call_fn_8982n#$12KLH?E
>?;
1?.
conv2d_24_input?????????dd
p

 
? "???????????
+__inference_sequential_8_layer_call_fn_9037n#$12KLH?E
>?;
1?.
conv2d_24_input?????????dd
p 

 
? "???????????
+__inference_sequential_8_layer_call_fn_9174e#$12KL??<
5?2
(?%
inputs?????????dd
p

 
? "???????????
+__inference_sequential_8_layer_call_fn_9195e#$12KL??<
5?2
(?%
inputs?????????dd
p 

 
? "???????????
"__inference_signature_wrapper_9068?#$12KLS?P
? 
I?F
D
conv2d_24_input1?.
conv2d_24_input?????????dd"=?:
8
activation_29'?$
activation_29?????????