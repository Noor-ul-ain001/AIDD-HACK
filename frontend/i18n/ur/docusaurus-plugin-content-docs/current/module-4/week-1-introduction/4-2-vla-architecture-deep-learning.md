---


sidebar_position: 2
difficulty: advanced


---
# 44

## ج a ج

یہ الل لِل لِل الحمول نِن ک ہ ایم ک ایک/کی ف ف aiaunaiul نِنورل نِنورل نِننن وور کے ، ، ، ، ، ، ، کے ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک

## ss یکھ n ے کے maua ص d

کے ذی ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ک ذی ذی ذی ذی ک ک ک ک ک ک ک ذی ذی ذی ذی ذی
- smauso کہ/کی bin ی aad ی ڈیپ llnnn گ گ ف ف ف ف ف ف ف ف a a گی a ہے a ہے ہے ہے
- aranssaurmr پ پ ی ی ی ی ف ف ف ف ف ف ف ف کے کے
- تونو کے کے ط ri ک ar ک v dri ی aa ف t icr یں۔
- ک a/کی taun یکی ai جز a ء ک v ssm جھیں۔ موولس مووسل السون
- تالسل ماما النگ کے لِل ی ِ ِsnasn Aaِnriشn کے بورے
- ک a/کی ک ک mausnillatahau ں کی کی کی کی ک ک ک ک ک ک
- عنا میں نِن ، اوسوور ، اوسووسوسسر اوسوسور اعدعمحر کے دفرمکون احر ق یسو سوسو

## ف aiuni ڈیش Na: نعورل ن یٹ وور کے اجز کے

### ون آnauswaurز

وژن انکوڈرز maus waus ی ی ی ی ک ک ک ک یہ یہ فن تعمیرات کا استعمال کرتے ہیں:

#####
۔
- ** اوسد **
.
- **Applications**: Early وی ایل اے ماڈلز, embedded systems
```python
import torch
import torch.nn کے طور پر nn

class VisionEncoder:
    def __init__(self, input_channels=3, feature_dim=512):
        super().__init__()
        self.conv_layers = nn.Sequential(
            nn.Conv2d(input_channels, 64, kernel_size=7, stride=2, padding=3),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1))  # Global average pooling
        )
        self.projection = nn.Linear(256, feature_dim)
    
    def forward(self, x):
        features = self.conv_layers(x)  # Shape: (batch, 256, 1, 1)
        features = features.view(features.size(0), -1)  # Flatten
        projected_features = self.projection(features)  # Shape: (batch, feature_dim)
        return projected_features
```
کیa
- ** آrکیٹیکچr **: پیچ ایمبیڈنگ + ٹرانسفارمر بلاکس
.
.
- **Applications**: Modern وی ایل اے ماڈلز, state-کا-کا/کی-art performance
```python
import torch
import torch.nn کے طور پر nn
import torch.nn.functional کے طور پر F

class VisionTransformer:
    def __init__(self, patch_size=16, num_channels=3, embed_dim=768, depth=12, num_heads=12):
        super().__init__()
        self.patch_size = patch_size
        self.embed_dim = embed_dim
        self.num_patches = (224 // patch_size) ** 2
        
        # Patch embedding layer
        self.patch_embed = nn.Conv2d(num_channels, embed_dim, 
                                     kernel_size=patch_size, stride=patch_size)
        
        # Positional embeddings
        self.pos_embed = nn.پیرامیٹر(torch.randn(1, self.num_patches + 1, embed_dim))
        self.cls_token = nn.پیرامیٹر(torch.randn(1, 1, embed_dim))
        
        # Transformer blocks
        self.blocks = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads) کے لیے _ میں range(depth)
        ])
        
        self.norm = nn.LayerNorm(embed_dim)
    
    def forward(self, x):
        B, C, H, W = x.shape
        
        # Convert image کو patches
        x = self.patch_embed(x)  # (B, embed_dim, num_patches_h, num_patches_w)
        x = x.flatten(2).transpose(1, 2)  # (B, num_patches, embed_dim)
        
        # Add class token
        cls_tokens = self.cls_token.expand(B, -1, -1)
        x = torch.cat([cls_tokens, x], dim=1)  # (B, num_patches+1, embed_dim)
        
        # Add positional embeddings
        x = x + self.pos_embed[:, :x.size(1)]
        
        # Apply transformer blocks
        کے لیے block میں self.blocks:
            x = block(x)
        
        x = self.norm(x)
        return x[:, 0]  # Return class token embedding

class TransformerBlock:
    def __init__(self, embed_dim, num_heads):
        super().__init__()
        self.attention = nn.MultiheadAttention(embed_dim, num_heads, batch_first=True)
        self.norm1 = nn.LayerNorm(embed_dim)
        self.norm2 = nn.LayerNorm(embed_dim)
        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, embed_dim * 4),
            nn.GELU(),
            nn.Linear(embed_dim * 4, embed_dim)
        )
    
    def forward(self, x):
        # Self-attention
        attn_out, _ = self.attention(x, x, x)
        x = x + attn_out
        x = self.norm1(x)
        
        # Feed-forward
        mlp_out = self.mlp(x)
        x = x + mlp_out
        x = self.norm2(x)
        
        return x
```
#####
- **ConvNeXt**: Convolutional layers کے ساتھ Transformer-style normalization
- **Swin Transformer**: Shifted windows کے لیے local-global attention
.

### ز بان anauchr ز

ز پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ نیچے پ پ پ پ پ پ پ پ پ پ

#####
.
- **GPT style**: Causal generation کا text اور ایکشنز
- **T5 style**: Encoder-decoder کے لیے text-کو-text tasks
```python
import torch
import torch.nn کے طور پر nn

class LanguageEncoder:
    def __init__(self, vocab_size=50257, embed_dim=768, max_seq_len=512, num_layers=12, num_heads=12):
        super().__init__()
        self.token_embedding = nn.Embedding(vocab_size, embed_dim)
        self.pos_embedding = nn.Embedding(max_seq_len, embed_dim)
        
        self.blocks = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads) کے لیے _ میں range(num_layers)
        ])
        
        self.ln_f = nn.LayerNorm(embed_dim)  # Final layer norm
    
    def forward(self, input_ids, attention_mask=None):
        # Embed tokens
        token_emb = self.token_embedding(input_ids)  # (B, seq_len, embed_dim)
        
        # Add positional embeddings
        seq_len = input_ids.size(1)
        pos_ids = torch.arange
        pos_emb = self.pos_embedding(pos_ids)  # (seq_len, embed_dim)
        pos_emb = pos_emb.unsqueeze(0)  # (1, seq_len, embed_dim)
        
        x = token_emb + pos_emb
        
        # Apply transformer blocks
        کے لیے block میں self.blocks:
            x = block(x)
        
        x = self.ln_f(x)
        return x  # (B, seq_len, embed_dim)
```
### Aisn ڈیک vaur ز

Aِn J کی کی کی کی کی کی کی کی کی کی کی کی j کی کی کی کی j کی

#### msslsl جگہیں
.
- **Operational Space**: اختتام-effector positions, rotations

#####
- **Primitive ایکشنز**: Pick, place, کھلا, بند
- **Symbolic ایکشنز**: اونچا-level commands
```python
import torch
import torch.nn کے طور پر nn

class ActionDecoder:
    def __init__(self, latent_dim=512, action_dim=7, hidden_dim=256):
        super().__init__()
        self.decoder = nn.Sequential(
            nn.Linear(latent_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # Normalize کو [-1, 1] کے لیے continuous control
        )
        
        # کے لیے different ایکشن types, we _MAYBE_ also رکھتے ہیں:
        # - Separate heads کے لیے position اور rotation
        # - Variational layers کے لیے uncertainty estimation
        # - Temporal prediction کے لیے trajectory planning
    
    def forward(self, latent_state):
        # Latent state سے multimodal fusion
        ایکشن = self.decoder(latent_state)
        return ایکشن
```
## توو ے ے کے کے ط ط r یقہ ک ک ک ک ک ک ک ک ک ک ک ک

###

خ ud ssaaut ہ ہ ہ ہ ہ ک j ہے ق ق ق ہے ہے ہے ہے ہے ہے ہے ہے ہے ہے ہے ہے ہے
```python
import torch
import torch.nn کے طور پر nn

class MultiHeadSelfAttention:
    def __init__(self, embed_dim, num_heads):
        super().__init__()
        assert embed_dim % num_heads == 0
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads
        
        self.qkv = nn.Linear(embed_dim, embed_dim * 3)
        self.proj = nn.Linear(embed_dim, embed_dim)
        
    def forward(self, x):
        B, N, C = x.shape
        qkv = self.qkv(x).reshape(B, N, 3, self.num_heads, self.head_dim)
        qkv = qkv.permute(2, 0, 3, 1, 4)  # (3, B, num_heads, N, head_dim)
        q, k, v = qkv[0], qkv[1], qkv[2]
        
        # Compute attention weights
        attn_weights = (q @ k.transpose(-2, -1)) * (self.head_dim ** -0.5)
        attn_weights = F.softmax(attn_weights, dim=-1)
        
        # Apply attention کو values
        output = attn_weights @ v
        output = output.transpose(1, 2).reshape(B, N, C)
        
        return self.proj(output)
```
### -

iSras ے یک یک یک یک یک یک یک یک یک ے ے ے ے ے ے ے ے ے ے ے ے ے ے ے کے کے کے کے کے کے کے کے کے کے کے کے کے کے ھ ھ ھ کے کے کے کے کے کے کے کے کے ھ ھ کے کے کے کے کے کے کے کے کے کے کے کے
```python
class CrossAttention:
    def __init__(self, embed_dim, num_heads):
        super().__init__()
        self.embed_dim = embed_dim
        self.num_heads = num_heads
        self.head_dim = embed_dim // num_heads
        
        # Separate projections کے لیے query (e.g., language) اور key/value (e.g., vision)
        self.query_proj = nn.Linear(embed_dim, embed_dim)
        self.kv_proj = nn.Linear(embed_dim, embed_dim * 2)
        self.output_proj = nn.Linear(embed_dim, embed_dim)
    
    def forward(self, query, key_value):
        B, N, C = query.shape
        _, M, _ = key_value.shape
        
        # Project query, key, اور value
        q = self.query_proj(query).reshape(B, N, self.num_heads, self.head_dim).transpose(1, 2)
        kv = self.kv_proj(key_value).reshape(B, M, 2, self.num_heads, self.head_dim).transpose(1, 2)
        k, v = kv[:, :, 0], kv[:, :, 1]
        
        # Compute cross-attention
        attn_weights = (q @ k.transpose(-2, -1)) * (self.head_dim ** -0.5)
        attn_weights = F.softmax(attn_weights, dim=-1)
        
        output = attn_weights @ v
        output = output.transpose(1, 2).reshape(B, N, C)
        
        return self.output_proj(output)
```
### mwli mwuxl thsos

کے کے کے کے کے کے کے ، ، ، ، ، ط ط ط ک ک ک ک ک ک ک ک ک ک ک ari ک ari ط ari ط rauchos ssiumaulumaut ط rauchus ssiuchus ssiulumaut ط ruuchuchuchus ط ط ط ط ک راؤچس جولوماؤٹ ک راؤچ ط ایری ط راؤچوس ط اے آر ط
```python
class MultimodalAttention:
    def __init__(self, embed_dim, num_heads):
        super().__init__()
        self.vision_to_lang = CrossAttention(embed_dim, num_heads)
        self.lang_to_vision = CrossAttention(embed_dim, num_heads)
        self.action_to_multimodal = CrossAttention(embed_dim, num_heads)
        
    def forward(self, vision_features, language_features, action_features=None):
        # Cross-attend vision اور language
        lang_with_vision = self.vision_to_lang(language_features, vision_features)
        vision_with_lang = self.lang_to_vision(vision_features, language_features)
        
        # کے لیے ایکشن prediction, attend کو multimodal context
        اگر action_features ہے نہیں None:
            action_with_context = self.action_to_multimodal(action_features, 
                torch.cat([lang_with_vision, vision_with_lang], dim=1))
            return vision_with_lang, lang_with_vision, action_with_context
        else:
            return vision_with_lang, lang_with_vision
```
## vla آ r کیٹیکچ r پیٹ rn

###

مااسسرن
```python
class UnifiedVLATransformer:
    def __init__(self, 
                 vocab_size=50257,
                 vision_patch_size=16,
                 embed_dim=768,
                 depth=12,
                 num_heads=12,
                 action_dim=7):
        super().__init__()
        
        # Modal-specific encoders
        self.vision_encoder = VisionTransformer(
            patch_size=vision_patch_size,
            embed_dim=embed_dim,
            depth=depth//2,  # Shallower vision encoder
            num_heads=num_heads
        )
        
        self.language_encoder = LanguageEncoder(
            vocab_size=vocab_size,
            embed_dim=embed_dim,
            num_layers=depth//2,
            num_heads=num_heads
        )
        
        # Cross-modal transformer layers
        self.cross_modal_layers = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads) کے لیے _ میں range(depth)
        ])
        
        # ایکشن prediction head
        self.action_head = nn.Sequential(
            nn.LayerNorm(embed_dim),
            nn.Linear(embed_dim, embed_dim // 2),
            nn.ReLU(),
            nn.Linear(embed_dim // 2, action_dim)
        )
        
        # Task identification head
        self.task_head = nn.Linear(embed_dim, 100)  # 100 possible tasks
    
    def forward(self, images, text_tokens, attention_mask=None):
        # Encode modalities separately
        vision_features = self.vision_encoder(images)  # (B, vision_seq_len, embed_dim)
        lang_features = self.language_encoder(text_tokens, attention_mask)  # (B, text_seq_len, embed_dim)
        
        # Concatenate modalities
        multimodal_input = torch.cat([vision_features, lang_features], dim=1)  # (B, combined_seq_len, embed_dim)
        
        # Process کے ساتھ cross-modal layers
        کے لیے layer میں self.cross_modal_layers:
            multimodal_input = layer(multimodal_input)
        
        # Extract representations کے لیے different heads
        # Use [CLS] token یا mean pooling کے لیے global representation
        pooled_features = multimodal_input.mean(dim=1)  # (B, embed_dim)
        
        # Generate predictions
        ایکشنز = self.action_head(pooled_features)
        task_pred = self.task_head(pooled_features)
        
        return {
            'ایکشنز': ایکشنز,
            'task': task_pred,
            'multimodal_features': multimodal_input
        }
```
###

رکوع ووبلکہ
```python
class VLAEcoderDecoder:
    def __init__(self, embed_dim=768, num_layers=12, num_heads=12, action_vocab_size=200):
        super().__init__()
        
        # Encoder کے لیے vision-language input
        self.encoder = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads) کے لیے _ میں range(num_layers // 2)
        ])
        
        # Decoder کے لیے ایکشن generation
        self.decoder = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads) کے لیے _ میں range(num_layers // 2)
        ])
        
        # ایکشن vocabulary projection
        self.action_projection = nn.Linear(embed_dim, action_vocab_size)
        
        # Cross-attention layer کے لیے encoder-decoder مواصلات
        self.enc_dec_attention = MultiHeadSelfAttention(embed_dim, num_heads)
    
    def forward(self, vision_lang_features, action_tokens=None):
        # Encode vision-language input
        encoded_features = vision_lang_features
        کے لیے layer میں self.encoder:
            encoded_features = layer(encoded_features)
        
        # Decode ایکشنز
        اگر action_tokens ہے نہیں None:
            # Teacher forcing کے دوران training
            decoded_features = self.process_decoder_tokens(action_tokens)
        else:
            # Autoregressive generation کے دوران inference
            decoded_features = self.autoregressive_decode(encoded_features)
        
        # Apply cross-attention between encoder اور decoder
        attended_features = self.enc_dec_attention(decoded_features, encoded_features)
        
        # Project کو ایکشن space
        action_logits = self.action_projection(attended_features)
        
        return action_logits
    
    def process_decoder_tokens(self, action_tokens):
        # Similar کو language model processing
        # Embed ایکشن tokens اور apply decoder layers
        pass
    
    def autoregressive_decode(self, encoded_features):
        # Autoregressive decoding similar کو GPT
        # Generate ایک ایکشن token پر ایک time
        pass
```
## موول ک موسل فی اووناٹن یک

### ذAtہ ئی ئی ئی ئی ئی ئی

ط rauch -ک v یکج a ک ri یں - an پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ پٹ.
```python
class EarlyFusionVLA:
    def __init__(self, vision_dim=2048, lang_dim=512, action_dim=7, hidden_dim=1024):
        super().__init__()
        # Project modalities کو common space
        self.vision_proj = nn.Linear(vision_dim, hidden_dim)
        self.lang_proj = nn.Linear(lang_dim, hidden_dim)
        
        # Combined processing
        self.fusion_network = nn.Sequential(
            nn.Linear(hidden_dim * 2, hidden_dim * 2),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.ReLU()
        )
        
        self.action_head = nn.Linear(hidden_dim, action_dim)
    
    def forward(self, vision_features, language_features):
        # Project کو common space
        vis_proj = self.vision_proj(vision_features)
        lang_proj = self.lang_proj(language_features)
        
        # Concatenate اور fuse
        fused = torch.cat([vis_proj, lang_proj], dim=-1)
        fused = self.fusion_network(fused)
        
        ایکشنز = self.action_head(fused)
        return ایکشنز
```
###

عمل کے ط rauch ک so آز Adan ہ ط ط vr پ r ، پھ آ آ آ آ آ آ آ آ آ آ آ آ آ آ آ
```python
class LateFusionVLA:
    def __init__(self, vision_dim=2048, lang_dim=512, action_dim=7):
        super().__init__()
        # Independent processing branches
        self.vision_branch = nn.Sequential(
            nn.Linear(vision_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 256)
        )
        
        self.language_branch = nn.Sequential(
            nn.Linear(lang_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 256)
        )
        
        # Fusion layer
        self.fusion = nn.Linear(256 * 2, 512)
        self.action_head = nn.Linear(512, action_dim)
    
    def forward(self, vision_features, language_features):
        # Process independently
        vis_out = self.vision_branch(vision_features)
        lang_out = self.language_branch(language_features)
        
        # Combine late
        combined = torch.cat([vis_out, lang_out], dim=-1)
        fused = self.fusion(combined)
        
        ایکشنز = self.action_head(fused)
        return ایکشنز
```
###

یک ss ے زی زی ہ ہ فی waua ئ n ٹ s - mautli ssucho ں ک خ خ خ lai:
```python
class HierarchicalFusionVLA:
    def __init__(self, embed_dim=768):
        super().__init__()
        # Initial modality processing
        self.vision_init = nn.Linear(2048, embed_dim)
        self.lang_init = nn.Linear(512, embed_dim)
        
        # کم-level fusion
        self.low_fusion = nn.MultiheadAttention(embed_dim, num_heads=8, batch_first=True)
        
        # Mid-level processing layers
        self.mid_layers = nn.ModuleList([
            TransformerBlock(embed_dim, 8) کے لیے _ میں range(4)
        ])
        
        # اونچا-level fusion
        self.high_fusion = nn.MultiheadAttention(embed_dim, num_heads=8, batch_first=True)
        
        # Final ایکشن prediction
        self.action_pred = nn.Linear(embed_dim, 7)
    
    def forward(self, vision_features, language_features):
        # Initial processing
        vis_processed = self.vision_init(vision_features).unsqueeze(1)  # Add sequence dimension
        lang_processed = self.lang_init(language_features).unsqueeze(1)
        
        # کم-level fusion
        fused_low, _ = self.low_fusion(vis_processed, lang_processed, lang_processed)
        
        # Mid-level processing
        mid_features = fused_low
        کے لیے layer میں self.mid_layers:
            mid_features = layer(mid_features)
        
        # اونچا-level fusion کے ساتھ context
        final_features, _ = self.high_fusion(mid_features, fused_low, fused_low)
        
        # Predict ایکشنز
        ایکشنز = self.action_pred(final_features.squeeze(1))
        return ایکشنز
```
## ٹ RAUNN گ پی RAA ڈی MI ز

###

s یکھیں - insan ی mauaa ہ roi ں کی taul ی d ک r یں:
```python
def behavioral_cloning_loss(model, batch):
    """
    Standard behavioral cloning objective
    """
    obs_images = batch['images']  # (B, C, H, W)
    obs_language = batch['language']  # (B, seq_len)
    ایکشنز = batch['expert_actions']  # (B, action_dim)
    
    predicted_actions = model(obs_images, obs_language)['ایکشنز']
    
    # Mean squared error کے لیے continuous ایکشنز
    bc_loss = F.mse_loss
    
    return bc_loss
```
### ک MA ک ssiun ے Sausna Ansan ی taauraat (rlhf)

پ Aal ی Sauchu ں ک v Buatr Bunan ے کے کے ئے ئے ئے Sasana ی taaaaaurat ک a ک a ک aasamal asri:
```python
def rlhf_loss(model, batch, reference_model=None):
    """
    Reinforcement Learning سے Human Feedback
    """
    states = batch['states']
    ایکشنز = batch['ایکشنز']
    rewards = batch['rewards']  # Human preference scores
    
    # Get log probabilities سے current model
    curr_log_probs = model.get_log_prob
    
    # Get log probabilities سے reference model (initial policy)
    اگر reference_model:
        ref_log_probs = reference_model.get_log_prob
        ratio = torch.exp(curr_log_probs - ref_log_probs)
    else:
        ratio = 1.0
    
    # Policy gradient loss
    pg_loss = -(ratio * rewards).mean()
    
    return pg_loss
```
### مستاعد سیسنا

nmaaund گی si یکھیں۔
```python
def contrastive_loss(vision_features, language_features, temperature=0.1):
    """
    Contrastive loss کو align vision اور language representations
    """
    # Compute similarity matrix
    sim_matrix = torch.matmul(vision_features, language_features.T) / temperature
    
    # Targets: diagonal elements چاہیے ہونا اونچا
    labels = torch.arange(len(vision_features)).کو(vision_features.device)
    
    # Cross entropy loss کہاں each image چاہیے match اس کا corresponding text
    loss_img = F.cross_entropy(sim_matrix, labels)
    loss_txt = F.cross_entropy(sim_matrix.T, labels)
    
    return (loss_img + loss_txt) / 2
```
## ک Mauswaunl talat

### maumur ی ​​کی ک ک ک ک ک

وِل اِسسل اِس مامامالز ک ک ​​ai فی ح ح ک ک ک mauchnil wssaaul کی ض crorat ہے:

.
- ** ماؤل مامورور
۔
```python
# Gradient checkpointing کو trade computation کے لیے memory
سے torch.utils.checkpoint import checkpoint

class MemoryEfficientVLA:
    def __init__(self, base_model):
        super().__init__()
        self.base_model = base_model
    
    def forward(self, images, text):
        def run_part1(imgs, txt):
            return self.base_model.encode_modalities(imgs, txt)
        
        def run_part2(fused):
            return self.base_model.decode_actions(fused)
        
        # Apply gradient checkpointing کو reduce memory usage
        fused_repr = checkpoint(run_part1, images, text)
        ایکشنز = checkpoint(run_part2, fused_repr)
        
        return ایکشنز
```
### JARTB کی Llimbaiئ کی ح ح ح ح ح ح ح

آپ کے کے mauna ز m ک o چ odair ط vr پ Man ے پ پ پ پ پ manaun ے t manaun ے ta maun ے lulmba ئی
```python
# Techniques کو handle لمبا sequences:
# 1. Sliding window attention
# 2. Sparse attention patterns
# 3. Linear attention approximations
# 4. Hierarchical processing

class SlidingWindowAttention:
    def __init__(self, embed_dim, num_heads, window_size=256):
        super().__init__()
        self.window_size = window_size
        self.attention = nn.MultiheadAttention(
            embed_dim, 
            num_heads, 
            batch_first=True
        )
    
    def forward(self, x):
        B, T, D = x.shape
        اگر T <= self.window_size:
            # Standard attention اگر sequence ہے چھوٹا enough
            return self.attention(x, x, x)[0]
        
        # Process میں overlapping windows
        outputs = []
        کے لیے میں میں range(0, T, self.window_size):
            end_idx = min
            window = x[:, میں:end_idx, :]
            
            window_out = self.attention(window, window, window)[0]
            outputs.append(window_out)
        
        # Combine outputs
        return torch.cat(outputs, dim=1)
```
## maa ڈ l یsaaln گ اوسن

کے ط ط کے کے کے ط ط پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ پ کے کے کے ط ط پ پ ی کے کے کے کے ط ط پ پ ی کے کے کے کے ط ط ی ی ی ی ی ی ط ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی ی نیچے ی

۔
.
- ** تچرب کے کے
.
```python
# مثال scaling law considerations
SCALING_RELATIONSHIPS = {
    'parameters_vs_performance': 'Generally follows power law کے ساتھ diminishing returns',
    'data_requirements': 'Scale roughly linearly کے ساتھ model size کے لیے optimal training',
    'compute_requirements': 'Scale superlinearly',
    'inference_latency': 'Increases linearly کے لیے feedforward models'
}
```
## خ LAA صہ

یہ ایلل لِل ، ماما ؤ ِ ہ ہ ہ so/کی ڈیپ lrnn گ llrnni گ llrnni گ آ crauri ز آ rairi ز vanaiuriaun vaun vaun luchnaui یج- ina maa ڈ- l:

1
2
3
4
5
6

فہی فہی فہی یہ آ آ آ آ آ ہے ہے ہے ہے ہے ہے ہے ہے کے کے کے کے کے کے ک ک ک ک j ک ک ، ، ، ، ، ، ، ، ، ، پ پ پ پ پ پ پ پ پ ، ، ، ، ، ، ، ، پ پ کے کے کے کے کے کے ہ ہ ہ m ہ ہ ہ ہ ہ ہ
