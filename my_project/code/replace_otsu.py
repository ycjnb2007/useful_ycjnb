import os

path = r'd:\fenkuandianlu\ADS\code\first\TC264_CrazyCircuit\Seekfree_TC264_test01\code\image_deal_best.c'
with open(path, 'r', encoding='gbk') as f:
    lines = f.readlines()

start = -1
for i, line in enumerate(lines):
    if 'uint8 getOSTUThreshold(void)' in line:
        start = i
        break

if start != -1:
    end = start
    braces = 0
    found_brace = False
    while end < len(lines):
        if '{' in lines[end]:
            braces += lines[end].count('{')
            found_brace = True
        if '}' in lines[end]:
            braces -= lines[end].count('}')
        
        end += 1
        if found_brace and braces == 0:
            break

    new_func = '''uint8 getOSTUThreshold(void) {
  getGrayscaleHistogram();
  uint32_t sum = 0, valueSum = 0;
  uint64_t sigma = 0, maxSigma = 0;

  uint8 min = 0, max = 255;
  min = minGray;
  max = maxGray;
  if (max < minThreshold)
    return minThreshold;
  if (min > maxThreshold)
    return maxThreshold;
  min = min < minGrayscale ? minGrayscale : min;
  max = max > maxGrayscale ? maxGrayscale : max;
  uint32_t lowSum[256] = {0};
  uint32_t lowValueSum[256] = {0};
  for (uint16_t i = min; i <= max; ++i) {
    sum += histogram[i];
    valueSum += histogram[i] * i;
    lowSum[i] = sum;
    lowValueSum[i] = valueSum;
  }
  for (uint16_t i = min; i <= max; ++i) {
    uint32_t w0 = lowSum[i];
    uint32_t w1 = sum - w0;

    if (w0 == 0 || w1 == 0) continue;

    // 采用定点数思想，通过 << 6 放大 64 倍，提升除法精度并彻底替代浮点数，且无溢出风险
    uint32_t u0 = (lowValueSum[i] << 6) / w0;
    uint32_t u1 = ((valueSum - lowValueSum[i]) << 6) / w1;
    uint32_t diff = (u0 > u1) ? (u0 - u1) : (u1 - u0);
    
    sigma = (uint64_t)w0 * w1 * diff * diff;
    
    if (sigma >= maxSigma) {
      maxSigma = sigma;
      nowThreshold = i;
    } else {
      break;
    }
  }
  static uint8_t last_Threshold = 120; // 前一帧阈值
  nowThreshold = nowThreshold < minThreshold ? minThreshold : nowThreshold;
  nowThreshold = nowThreshold > maxThreshold ? maxThreshold : nowThreshold;
  // 消除小数，同比例整型相乘并整除
  nowThreshold = (uint8_t)((7 * nowThreshold + 3 * last_Threshold) / 10);
  last_Threshold = nowThreshold;
  return nowThreshold;
}
'''
    with open(path, 'w', encoding='gbk') as f:
        f.writelines(lines[:start])
        f.write(new_func)
        f.writelines(lines[end:])
    print("Replace success.")
else:
    print("Could not find function.")
