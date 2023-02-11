import torch
import torch.nn as nn

'''
Batch normalization does not use a scale and bias (affine = False) here so it is easy to convert it to fit the tiny_dnn framework.
Note that the tiny_dnn reimplementation does not include the upsamping layer.
'''
    
class conv_bn_relu(nn.Module): #affine=False for batch norm
    def __init__(self, in_channels, out_channels, kernel_size, stride, padding, activation=True):
        super(conv_bn_relu, self).__init__()
        
        self.net = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding),
            nn.BatchNorm2d(out_channels, affine=False),
            nn.ReLU(True)
        )
        
    def forward(self, x):
        return self.net(x)


class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        
        self.net = nn.Sequential(
            conv_bn_relu(1, 8, 3, 2, 1), # 320x240 -> 160x120

            conv_bn_relu(8, 16, 3, 2, 1), # 160x120 -> 80x60
            conv_bn_relu(16, 16, 3, 1, 1),
            
            conv_bn_relu(16, 24, 3, 2, 1), # 80x60 -> 40x30
            conv_bn_relu(24, 24, 3, 1, 1),
            
            conv_bn_relu(24, 32, 3, 2, 1), # 40x30 -> 20x15
            conv_bn_relu(32, 32, 3, 1, 1),
            conv_bn_relu(32, 32, 3, 1, 1),
            
            nn.Conv2d(32, 1, 3, 1, 1),
            
            nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
        )
        
    def forward(self, x):
        return self.net(x)


class transpose_layer(nn.Module):
    '''
    the purpose of this layer is to rearrange the input plane so CPUs can do
    vectorization across columns.  the output plane has the columns in rows, and
    vectorization can do a dot product in one CPU instruction
    '''
    def forward(self, x):
        return x.transpose(-2, -1)


class JNN1(nn.Module):
    '''
    the aim of this network is to enable CPUs to use vectorization, which does a
    dot product in one CPU instruction.  The v6 CPUs support SSE4.2.  SSE4.1
    includes the dot product instruction DPPS, but we also need 16-byte
    (4-float) aligned kernels and input planes.  DPPS may not be the fastest,
    but MULPS has the same alignment requirement:
    https://stackoverflow.com/a/4121295/192798

    without alignment (3x1 kernel, 2x_ stride, 1x_ padding):
    piii iiii iiii
    kkk
      kk k
         kkk
           kk k
    i'm not sure where the padding is, so it could be like this:
    p iiii iiii iiii
    k kk
       kkk
         k kk
            kkk

    with alignment (4x_ kernel, 4x_ stride, 0x_ padding):
    iiii iiii iiii
    kkkk
         kkkk
              kkkk
    '''
    def __init__(self):
        super().__init__()
        
        self.net = nn.Sequential(
          # 320 x 240
            conv_bn_relu(1, 8, (3, 4), (2, 4), (1, 0)),
          #  80 x 120

            conv_bn_relu(8, 16, 3, (2, 1), (1, 1)),
          #  80 x  60
            conv_bn_relu(16, 16, 3, 1, 1),
            
            conv_bn_relu(16, 24, 3, 2, 1),
          #  40 x  30
            conv_bn_relu(24, 24, 3, 1, 1),
            
            conv_bn_relu(24, 32, 3, 2, 1),
          #  20 x  15
            conv_bn_relu(32, 32, 3, 1, 1),
            conv_bn_relu(32, 32, 3, 1, 1),
            
            nn.Conv2d(32, 1, 3, 1, 1),
            
            nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
        )
        
    def forward(self, x):
        return self.net(x)


# JNN1 with 8x3 kernel
class JNN2(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 8), (2, 4), (1, 2)),
      #  80 x 120

      conv_bn_relu(8, 16, 3, (2, 1), (1, 1)),
      #  80 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, 2, 1),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN2 with fewer channels
class JNN3(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 8), (2, 4), (1, 2)),
      #  80 x 120

      conv_bn_relu(8, 8, 3, (2, 1), (1, 1)),
      #  80 x  60
      conv_bn_relu(8, 16, 3, 1, 1),
      
      conv_bn_relu(16, 24, 3, 2, 1),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),
      
      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),
      
      nn.Conv2d(32, 1, 3, 1, 1),
      
      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


#JNN1 but with the second layer vectorized
class JNN4(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 4), (2, 4), (1, 0)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN4 with 8x3 kernel
class JNN5(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN5 with fewer channels
class JNN6(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 8, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(8, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# more vectorization
class JNN7(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 8), (2, 4), (1, 2)),
      #  80 x 120
      conv_bn_relu(8, 8, 3, 1, 1),

      transpose_layer(),
      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      transpose_layer(),
      #  40 x  30
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN1 & JNN4 seem much faster, so try JNN7 with 4x3 kernels
class JNN8(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 4), (2, 4), (1, 0)),
      #  80 x 120
      conv_bn_relu(8, 8, 3, 1, 1),

      transpose_layer(),
      conv_bn_relu(8, 16, (3, 4), (2, 4), (1, 0)),
      transpose_layer(),
      #  40 x  30
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN3 has a good balance of loss and time, and the fewest parameters
# so let's try a bigger kernel in the first layer
class JNN9(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 12), (2, 4), (1, 4)), # 296p
      #  80 x 120

      conv_bn_relu(8, 8, 3, (2, 1), (1, 1)), #584p
      #  80 x  60
      conv_bn_relu(8, 16, 3, 1, 1), #1168p
      
      conv_bn_relu(16, 24, 3, 2, 1), #3480p
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1), #5208p
      
      conv_bn_relu(24, 32, 3, 2, 1), #6944p
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1), #9248p
      conv_bn_relu(32, 32, 3, 1, 1), #9248p
      
      nn.Conv2d(32, 1, 3, 1, 1), #289p
      
      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN1 and JNN4 have great times
# JNN1 has terrible loss
# so let's try JNN4 with more channels
class JNN10(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 24, (3, 4), (2, 4), (1, 0)),
      #  40 x  60
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN8 is fast, so try JNN8 with more channels
class JNN11(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 16, (3, 4), (2, 4), (1, 0)),
      #  80 x 120
      conv_bn_relu(16, 16, 3, 1, 1),

      transpose_layer(),
      conv_bn_relu(16, 32, (3, 4), (2, 4), (1, 0)),
      transpose_layer(),
      #  40 x  30
      conv_bn_relu(32, 32, 3, 1, 1),

      conv_bn_relu(32, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN5 with 32 upsample
class JNN12(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),

      conv_bn_relu(32, 48, 3, (1, 2), 1),
      #  10 x  15
      conv_bn_relu(48, 48, 3, 1, 1),
      conv_bn_relu(48, 48, 3, 1, 1),

      nn.Conv2d(48, 1, 3, 1, 1),

      nn.Upsample(scale_factor=(16, 32), mode='bilinear', align_corners=True)   
    )


# JNN5 with 8 upsample
class JNN13(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),
      conv_bn_relu(24, 32, 3, 1, 1),

      nn.Conv2d(32, 1, 3, 1, 1),

      nn.Upsample(scale_factor=8, mode='bilinear', align_corners=True)   
    )


# trying to use pixelshuffle
class JNN14(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu( 1,  8, (3, 8), (2, 4), (1, 2)),
      #  80 x 120
      conv_bn_relu( 8,  8, (3, 3), (1, 1), (1, 1)),
      conv_bn_relu( 8,  8, (3, 3), (1, 1), (1, 1)),
      conv_bn_relu( 8,  8, (3, 3), (1, 1), (1, 1)),

      conv_bn_relu( 8, 64, (7, 4), (4, 2), (2, 1)),
      #  40 x  30

      nn.PixelShuffle(8),
      # 320 x 240
    )


# trying to use pixelshuffle
class JNN15(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu( 1,  24, (3, 8), (3, 8), (1, 2)),
      #  40 x  80
      conv_bn_relu(24, 160, (7, 4), (5, 4), (1, 0)),
      #  10 x  16
      nn.PixelShuffle(4),
      #  40 x  64
      nn.Conv2d(   10,  10, (7, 3), (1, 1), (3, 1)),
      nn.Conv2d(   10,  10, (3, 7), (1, 1), (1, 3)),
      nn.Conv2d(   10,  10, (7, 3), (1, 1), (3, 1)),
      nn.Conv2d(   10,   1, (3, 7), (1, 1), (1, 3)),
      nn.Upsample(scale_factor=(3.75, 8), mode='bilinear', align_corners=True)
      # 320 x 240
    )


# JNN5 with pixeshuffle
class JNN16(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),
      conv_bn_relu(32, 32, 3, 1, 1),

      nn.PixelShuffle(4),
      #  80 x  60
      nn.Conv2d(2, 1, 3, 1, 1),

      nn.Upsample(scale_factor=4, mode='bilinear', align_corners=True)   
    )


# JNN5 further downsampled
class JNN17(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      conv_bn_relu(16, 16, 3, 1, 1),

      conv_bn_relu(16, 24, 3, (2, 1), (1, 1)),
      #  40 x  30
      conv_bn_relu(24, 24, 3, 1, 1),

      conv_bn_relu(24, 32, 3, 2, 1),
      #  20 x  15
      conv_bn_relu(32, 32, 3, 1, 1),

      conv_bn_relu(32, 64, (3, 8), (3, 4), (0, 2)),
      #   5 x   5
      conv_bn_relu(64, 64, 3, 1, 1),
      conv_bn_relu(64, 64, 3, 1, 1),

      nn.PixelShuffle(4),
      #  20 x  20
      nn.Conv2d(4, 1, 3, 1, 1),

      nn.Upsample(scale_factor=(12, 16), mode='bilinear', align_corners=True)   
    )


# JNN5 with bigger kernels
class JNN18(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, (3, 4), 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(16, 16, (3, 4), 1, 1),

      nn.ZeroPad2d((0,1,0,0)),
      conv_bn_relu(16, 24, (3, 4), (2, 1), (1, 1)),
      #  40 x  30
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(24, 24, (3, 4), 1, 1),

      conv_bn_relu(24, 32, (3, 4), 2, 1),
      #  20 x  15
      nn.ZeroPad2d((0,1,0,0)),
      conv_bn_relu(32, 32, (3, 4), 1, 1),
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(32, 32, (3, 4), 1, 1),

      nn.ZeroPad2d((0,1,0,0)),
      nn.Conv2d(32, 1, (3, 4), 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )


# JNN5 with bigger kernels
class JNN19(JNN1):
  def __init__(self):
    super().__init__()

    self.net = nn.Sequential(
      # 320 x 240
      conv_bn_relu(1, 8, 3, 2, 1),
      # 160 x 120

      conv_bn_relu(8, 16, (3, 8), (2, 4), (1, 2)),
      #  40 x  60
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(16, 16, (3, 4), 1, 1),

      nn.ZeroPad2d((0,1,0,0)),
      conv_bn_relu(16, 24, (3, 4), (2, 1), (1, 1)),
      #  40 x  30
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(24, 24, (3, 4), 1, 1),

      conv_bn_relu(24, 32, (3, 4), 2, 1),
      #  20 x  15
      nn.ZeroPad2d((0,1,0,0)),
      conv_bn_relu(32, 32, (3, 4), 1, 1),
      nn.ZeroPad2d((1,0,0,0)),
      conv_bn_relu(32, 32, (3, 4), 1, 1),

      nn.ZeroPad2d((0,1,0,0)),
      nn.Conv2d(32, 1, (3, 4), 1, 1),

      nn.Upsample(scale_factor=16, mode='bilinear', align_corners=True)   
    )
'''
stride=1, kernel=3, pad=1
stride=1, kernel=4, pad=1.5 <== have to use nn.ZeroPad2d((l,r,t,b))
stride=1, kernel=7, pad=3
stride=1, kernel=8, pad=3.5 <== have to use nn.ZeroPad2d((l,r,t,b))
stride=1, kernel=n, pad=n // 2 (n must be odd)
stride=2, kernel=3, pad=1
stride=2, kernel=4, pad=1
stride=2, kernel=8, pad=3
stride=2, kernel=n, pad=(n-1) // 2
stride=3, kernel=3, pad=0
stride=3, kernel=4, pad=1
stride=3, kernel=8, pad=3
stride=3, kernel=n, pad=(n-2) // 2
stride=s, kernel=k, pad=(k - s+1) // 2
2 * p < k <= 2 * p + s
(k - s) / 2 <= p < k / 2
p = (k - s + 1) // 2
'''