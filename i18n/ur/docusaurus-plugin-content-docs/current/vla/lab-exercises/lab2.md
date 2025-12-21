---
title: لیب 2 - VLA ماڈلز کی Fine-tuning
sidebar_position: 22
---

# لیب 2: مخصوص روبوٹک ٹاسکس کے لیے VLA ماڈلز کی Fine-tuning

## مقصد

اس لیب میں آپ سیکھیں گے کہ پہلے سے تربیت یافتہ Vision-Language-Action (VLA) ماڈل کو اپنے ماحول میں مخصوص روبوٹک ٹاسکس کے لیے کیسے fine-tune کیا جائے۔

## سیکھنے کے مقاصد

اس لیب کے بعد آپ:
- VLA ماڈل fine-tuning کے لیے ٹاسک-مخصوص ڈیٹا سیٹ تیار کر سکیں گے
- Fine-tuning procedures کنفیگر اور چلا سکیں گے
- Fine-tuning سے کارکردگی میں بہتری کا جائزہ لے سکیں گے
- ریئل ٹائم روبوٹک کنٹرول کے لیے ماڈل inference آپٹمائز کر سکیں گے

## پیش شرائط

- ہفتہ 10-11 سے VLA ماڈلز کی سمجھ
- لیب 1 میں بنائے گئے VLA سسٹم کا تجربہ
- Deep learning frameworks (PyTorch/TensorFlow) کا علم
- مخصوص روبوٹک ٹاسکس کے لیے labeled ڈیٹا

## ضروری آلات

- NVIDIA GPU والا کمپیوٹر (RTX 3080 یا اس کے برابر)
- کیمرہ اور manipulation صلاحیت والا روبوٹ
- ٹاسک demonstrations/annotations کا ڈیٹا سیٹ
- 50GB+ disk space

## لیب کے مراحل

### مرحلہ 1: ڈیٹا سیٹ کی تیاری

1. اپنے مخصوص ٹاسک کے لیے ڈیٹا جمع کریں:
   - آبجیکٹ manipulation: مخصوص اشیاء اٹھانے کی demonstrations
   - Navigation: زبان کے احکامات کے ساتھ path following
   - Assembly: بصری اور زبان context کے ساتھ multi-step manipulation

2. ڈیٹا سیٹ کی ساخت:
   ```bash
   dataset/
   ├── train/
   │   ├── images/
   │   ├── language_commands.json
   │   └── actions.json
   ├── validation/
   │   ├── images/
   │   ├── language_commands.json
   │   └── actions.json
   └── test/
       ├── images/
       ├── language_commands.json
       └── actions.json
   ```

### مرحلہ 2: ماڈل آرکیٹیکچر کا انتخاب

اپنے ٹاسک کے مطابق pre-trained VLA ماڈل منتخب کریں:

```python
import torch
from transformers import CLIPVisionModel, CLIPTextModel
import torch.nn as nn

class TaskSpecificVLA(nn.Module):
    """
    ٹاسک-مخصوص VLA ماڈل جو pre-trained ماڈل کو
    خصوصی روبوٹک ٹاسکس کے لیے adapt کرتا ہے
    """
    def __init__(self, base_model_name="openai/clip-vit-base-patch32"):
        super(TaskSpecificVLA, self).__init__()

        # Pre-trained vision اور language encoders لوڈ کریں
        self.vision_encoder = CLIPVisionModel.from_pretrained(base_model_name)
        self.text_encoder = CLIPTextModel.from_pretrained(base_model_name)

        # ٹاسک-مخصوص action head
        self.action_head = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(256, 6)  # 6DOF action space
        )

        # Pre-trained components freeze کریں
        for param in self.vision_encoder.parameters():
            param.requires_grad = False
        for param in self.text_encoder.parameters():
            param.requires_grad = False

    def forward(self, pixel_values, input_ids, attention_mask):
        # Visual features encode کریں
        vision_outputs = self.vision_encoder(pixel_values=pixel_values)
        visual_features = vision_outputs.pooler_output

        # Text features encode کریں
        text_outputs = self.text_encoder(
            input_ids=input_ids,
            attention_mask=attention_mask
        )
        text_features = text_outputs.pooler_output

        # Features combine کریں
        combined_features = torch.cat([visual_features, text_features], dim=1)

        # Actions predict کریں
        actions = self.action_head(combined_features)
        return actions
```

### مرحلہ 3: Fine-tuning کنفیگریشن

```python
from torch.utils.data import DataLoader
import torch.optim as optim

# ڈیٹا لوڈ کریں
train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True)

# ماڈل initialize کریں
model = TaskSpecificVLA()

# Loss function اور optimizer
criterion = nn.MSELoss()
optimizer = optim.AdamW([
    {'params': model.action_head.parameters(), 'lr': 1e-4},
    {'params': model.vision_encoder.encoder.layers[-2:].parameters(), 'lr': 5e-6},
], weight_decay=0.01)

# Learning rate scheduler
scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.9)
```

### مرحلہ 4: Fine-tuning چلائیں

```python
def fine_tune_vla_model(model, train_loader, val_loader, num_epochs=20):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    model.train()
    best_val_loss = float('inf')

    for epoch in range(num_epochs):
        epoch_loss = 0.0
        for batch_idx, (images, texts, actions) in enumerate(train_loader):
            images = images.to(device)
            actions = actions.to(device)

            optimizer.zero_grad()
            predicted_actions = model(images, texts['input_ids'], texts['attention_mask'])

            loss = criterion(predicted_actions, actions)
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()

        # Validation
        model.eval()
        val_loss = evaluate_model(model, val_loader)
        scheduler.step()

        print(f'Epoch {epoch}, Train Loss: {epoch_loss/len(train_loader):.6f}, Val Loss: {val_loss:.6f}')

        # بہترین ماڈل save کریں
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), 'best_vla_model.pth')

        model.train()

# Fine-tuning چلائیں
fine_tune_vla_model(model, train_loader, val_loader)
```

### مرحلہ 5: ماڈل کی جانچ اور انٹیگریشن

1. Test set پر ماڈل کا جائزہ لیں
2. Fine-tuned ماڈل کو ROS نوڈ میں شامل کریں
3. Generic اور fine-tuned ماڈلز کا موازنہ کریں

### مرحلہ 6: Deployment کے لیے آپٹمائزیشن

```python
# Quantization لگائیں
import torch.quantization as quantization

model.eval()
model.qconfig = quantization.get_default_qconfig('fbgemm')
quantized_model = quantization.prepare(model, inplace=False)
quantized_model = quantization.convert(quantized_model, inplace=False)
```

## لیب رپورٹ

جمع کروائیں:
1. **ڈیٹا سیٹ کی تفصیل**: Fine-tuning کے لیے استعمال شدہ ڈیٹا
2. **Fine-tuning پروسیس**: Hyperparameters، training curves
3. **کارکردگی کا موازنہ**: Generic vs fine-tuned ماڈل
4. **انٹیگریشن کے نتائج**: روبوٹ کی کارکردگی میں بہتری

## عام مسائل

| مسئلہ | حل |
|-------|-----|
| Overfitting | زیادہ regularization، متنوع ڈیٹا |
| سست inference | Quantization، ماڈل سادہ کریں |
| Catastrophic forgetting | Learning rate annealing |
| کم ڈیٹا | Data augmentation، synthetic ڈیٹا |

## خلاصہ

اس لیب میں آپ نے سیکھا کہ VLA ماڈلز کو مخصوص روبوٹک ٹاسکس کے لیے کیسے fine-tune کیا جائے، جس سے کارکردگی میں نمایاں بہتری آتی ہے۔
