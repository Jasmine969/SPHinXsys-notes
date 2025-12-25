geoparticle生成reload文件即可。pandas.DataFrame().to_xml()

```python
df.to_xml('name_rld.xml', 
          root_name='particles',
          row_name='particle',
          index=False,
          attr_cols=df.columns.tolist(),
          xml_declaration=False)  # 关键参数：禁用 XML 声明
```

